// STATE MACHINE
module fsm#(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32,
    parameter ID_WIDTH = 4
)(
    input  logic clk,
    input  logic reset,
    
    // Request FIFO interface
    input  logic req_fifo_empty,
    output logic req_fifo_rd_en,
    input  logic [31+1+4+8+2+3+3-1:0] req_fifo_rd_data, // addr+write+id+len+burst+size+prot
    
    // Write Data FIFO interface
    input  logic wdata_fifo_empty,
    output logic wdata_fifo_rd_en,
    input  logic [31+4+1+4-1:0] wdata_fifo_rd_data, // data+strb+last+id
    
    // Read Data FIFO interface
    input  logic rdata_fifo_full,
    output logic rdata_fifo_wr_en,
    output logic [31+4+1+2-1:0] rdata_fifo_wr_data, // data+id+last+resp
    
    // Write Response interface
    output logic bresp_valid,
    input  logic bresp_ready,
    output logic [3:0] bresp_id,
    output logic [1:0] bresp_resp,
    
    // APB Master interface
    output logic apb_psel,
    output logic apb_penable,
    output logic apb_pwrite,
    output logic [ADDR_WIDTH-1:0] apb_paddr,
    output logic [DATA_WIDTH-1:0] apb_pwdata,
    output logic [3:0] apb_pstrb,
    output logic [2:0] apb_pprot,
    input  logic [DATA_WIDTH-1:0] apb_prdata,
    input  logic apb_pready,
    input  logic apb_pslverr
);

typedef enum logic [3:0] 
{
    IDLE,
    FETCH_CMD,
    WAIT_CMD_DATA,    
    FETCH_DATA,
    WAIT_WRITE_DATA,  
    APB_SETUP,
    APB_ACCESS,
    WRITE_RESP,
    READ_STORE,
    READ_RESP,
    ERROR
} state_t;

state_t state, next_state;

// Registers to store unpacked FIFO data
logic [ADDR_WIDTH-1:0] cmd_addr_reg;
logic cmd_write_reg;
logic [ID_WIDTH-1:0] cmd_id_reg;
logic [7:0] cmd_len_reg;
logic [1:0] cmd_burst_reg;
logic [2:0] cmd_size_reg;
logic [2:0] cmd_prot_reg;

logic [DATA_WIDTH-1:0] wdata_reg;
logic [3:0] wstrb_reg;
logic wlast_reg;
logic [ID_WIDTH-1:0] wdata_id_reg;

logic [7:0] timeout_counter;
logic error_flag;

// Unpack request FIFO data
wire [ADDR_WIDTH-1:0] req_addr = req_fifo_rd_data[51:20];
wire req_write = req_fifo_rd_data[19];
wire [ID_WIDTH-1:0] req_id = req_fifo_rd_data[18:15];
wire [7:0] req_len = req_fifo_rd_data[14:7];
wire [1:0] req_burst = req_fifo_rd_data[6:5];
wire [2:0] req_size = req_fifo_rd_data[4:2];
wire [2:0] req_prot = req_fifo_rd_data[1:0];

// Unpack write data FIFO data
wire [DATA_WIDTH-1:0] wfifo_data = wdata_fifo_rd_data[39:8];
wire [3:0] wfifo_strb = wdata_fifo_rd_data[7:4];
wire wfifo_last = wdata_fifo_rd_data[3];
wire [ID_WIDTH-1:0] wfifo_id = wdata_fifo_rd_data[2:0];

// Default signals
always_comb begin
    req_fifo_rd_en    = 0;
    wdata_fifo_rd_en  = 0;
    rdata_fifo_wr_en  = 0;
    rdata_fifo_wr_data = '0;
    apb_psel          = 0;
    apb_penable       = 0;
    apb_pwrite        = 0;
    apb_paddr         = '0;
    apb_pwdata        = '0;
    apb_pstrb         = '0;
    apb_pprot         = '0;
    bresp_valid       = 0;
    bresp_id          = '0;
    bresp_resp        = 2'b00;
    error_flag        = 0;
    next_state        = state;

    case(state)
        IDLE: begin
            if (!req_fifo_empty) begin
                next_state = FETCH_CMD;
            end
        end

        FETCH_CMD: begin
            req_fifo_rd_en = 1;
            next_state = WAIT_CMD_DATA;
        end

        WAIT_CMD_DATA: begin
            if (cmd_write_reg) begin // WRITE
                if (!wdata_fifo_empty) begin
                    next_state = FETCH_DATA;
                end else begin
                    error_flag = 1;
                    next_state = ERROR;
                end
            end else begin // READ
                next_state = APB_SETUP;
            end
        end

        FETCH_DATA: begin
            wdata_fifo_rd_en = 1;
            next_state = WAIT_WRITE_DATA;
        end

        WAIT_WRITE_DATA: begin
            next_state = APB_SETUP;
        end

        APB_SETUP: begin
            apb_psel    = 1;
            apb_penable = 0;
            apb_paddr   = cmd_addr_reg;
            apb_pwrite  = cmd_write_reg;
            apb_pwdata  = cmd_write_reg ? wdata_reg : '0;
            apb_pstrb   = cmd_write_reg ? wstrb_reg : '0;
            apb_pprot   = cmd_prot_reg;
            next_state  = APB_ACCESS;
        end

        APB_ACCESS: begin
            apb_psel    = 1;
            apb_penable = 1;
            apb_paddr   = cmd_addr_reg;
            apb_pwrite  = cmd_write_reg;
            apb_pwdata  = cmd_write_reg ? wdata_reg : '0;
            apb_pstrb   = cmd_write_reg ? wstrb_reg : '0;
            apb_pprot   = cmd_prot_reg;
            
            // Check timeout
            if (timeout_counter == 8'hFF) begin
                error_flag = 1;
                next_state = ERROR;
            end else if (apb_pready) begin
                if (apb_pslverr) begin
                    error_flag = 1;
                    next_state = ERROR;
                end else if (cmd_write_reg) begin // WRITE
                    next_state = WRITE_RESP;
                end else begin // READ
                    next_state = READ_STORE;
                end
            end
        end

        WRITE_RESP: begin
            bresp_valid = 1;
            bresp_id = cmd_id_reg;
            bresp_resp = error_flag ? 2'b10 : 2'b00;
            if (bresp_ready) begin
                next_state = IDLE;
            end
        end

        READ_STORE: begin
            if (!rdata_fifo_full) begin
                rdata_fifo_wr_en = 1;
                rdata_fifo_wr_data = {apb_prdata, cmd_id_reg, 1'b1, 2'b00}; // data+id+last+resp
                next_state = IDLE;
            end else begin
                error_flag = 1;
                next_state = ERROR;
            end
        end

        ERROR: begin
            error_flag = 1;
            bresp_valid = 1;
            bresp_id = cmd_id_reg;
            bresp_resp = 2'b10; // SLVERR
            if (bresp_ready) begin
                next_state = IDLE;
            end
        end

        default: next_state = IDLE;
    endcase
end

always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        state <= IDLE;
        timeout_counter <= '0;
        cmd_addr_reg <= '0;
        cmd_write_reg <= '0;
        cmd_id_reg <= '0;
        cmd_len_reg <= '0;
        cmd_burst_reg <= '0;
        cmd_size_reg <= '0;
        cmd_prot_reg <= '0;
        wdata_reg <= '0;
        wstrb_reg <= '0;
        wlast_reg <= '0;
        wdata_id_reg <= '0;
    end else begin
        state <= next_state;
        
        // Timeout counter
        if (state == APB_ACCESS && !apb_pready) begin
            timeout_counter <= timeout_counter + 1;
        end else begin
            timeout_counter <= '0;
        end
        
        // Store command data when fetched
        if (state == FETCH_CMD) begin
            cmd_addr_reg <= req_addr;
            cmd_write_reg <= req_write;
            cmd_id_reg <= req_id;
            cmd_len_reg <= req_len;
            cmd_burst_reg <= req_burst;
            cmd_size_reg <= req_size;
            cmd_prot_reg <= req_prot;
        end
        
        // Store write data when fetched
        if (state == FETCH_DATA) begin
            wdata_reg <= wfifo_data;
            wstrb_reg <= wfifo_strb;
            wlast_reg <= wfifo_last;
            wdata_id_reg <= wfifo_id;
        end
    end
end

endmodule