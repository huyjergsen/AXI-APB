// =============================================================================
// Module      : fsm
// Project     : AXI-to-APB Bridge IP
// Description : Bridge control FSM. Dequeues commands and write-data from their
//               respective FIFOs, drives APB4 transactions (single and burst),
//               and enqueues read-data or write-response back to the AXI layer.
//               Handles address errors and APB timeout with safe burst abort.
// -----------------------------------------------------------------------------
// Parameters:
//   DATA_WIDTH       - Data bus width                       (default: 32)
//   ADDR_WIDTH       - Address bus width                    (default: 32)
//   ID_WIDTH         - AXI transaction ID width             (default:  4)
//   ADDR_BASE        - Base address of valid range          (default: 0x1000_0000)
//   ADDR_SIZE        - Size of valid address range (bytes)  (default: 0x0010_0000)
//   REQ_FIFO_WIDTH   - Width of the request FIFO entry      (computed)
//   WDATA_FIFO_WIDTH - Width of the write-data FIFO entry   (computed)
//   RDATA_FIFO_WIDTH - Width of the read-data FIFO entry    (computed)
// =============================================================================

module fsm #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32,
    parameter ID_WIDTH   = 4,
    parameter ADDR_BASE  = 32'h1000_0000,
    parameter ADDR_SIZE  = 32'h0010_0000,

    // Derived FIFO widths. Exposed as parameters so Quartus can compute port
    // widths at elaboration time (localparam cannot be used in port declarations).
    parameter REQ_FIFO_WIDTH   = ADDR_WIDTH + 1 + ID_WIDTH + 8 + 2 + 3 + 3,
    parameter WDATA_FIFO_WIDTH = DATA_WIDTH + DATA_WIDTH/8 + 1 + ID_WIDTH,
    parameter RDATA_FIFO_WIDTH = DATA_WIDTH + ID_WIDTH + 1 + 2
)(
    input  logic clk,
    input  logic reset,

    // -------------------------------------------------------------------------
    // Request FIFO Interface
    // -------------------------------------------------------------------------
    input  logic                        req_fifo_empty,
    output logic                        req_fifo_rd_en,
    input  logic [REQ_FIFO_WIDTH-1:0]   req_fifo_rd_data,

    // -------------------------------------------------------------------------
    // Write Data FIFO Interface
    // -------------------------------------------------------------------------
    input  logic                        wdata_fifo_empty,
    output logic                        wdata_fifo_rd_en,
    input  logic [WDATA_FIFO_WIDTH-1:0] wdata_fifo_rd_data,

    // -------------------------------------------------------------------------
    // Read Data FIFO Interface
    // -------------------------------------------------------------------------
    input  logic                        rdata_fifo_full,
    output logic                        rdata_fifo_wr_en,
    output logic [RDATA_FIFO_WIDTH-1:0] rdata_fifo_wr_data,

    // -------------------------------------------------------------------------
    // Write Response Interface (to AXI layer)
    // -------------------------------------------------------------------------
    output logic                        bresp_valid,
    input  logic                        bresp_ready,
    output logic [ID_WIDTH-1:0]         bresp_id,
    output logic [1:0]                  bresp_resp,

    // -------------------------------------------------------------------------
    // APB4 Master Interface
    // -------------------------------------------------------------------------
    output logic                    apb_psel,
    output logic                    apb_penable,
    output logic                    apb_pwrite,
    output logic [ADDR_WIDTH-1:0]   apb_paddr,
    output logic [DATA_WIDTH-1:0]   apb_pwdata,
    output logic [DATA_WIDTH/8-1:0] apb_pstrb,
    output logic [2:0]              apb_pprot,
    input  logic [DATA_WIDTH-1:0]   apb_prdata,
    input  logic                    apb_pready,
    input  logic                    apb_pslverr
);

// =============================================================================
// State Encoding
// =============================================================================
typedef enum logic [3:0] {
    IDLE,             // Wait for request in FIFO
    FETCH_CMD,        // Issue FIFO read enable for command
    WAIT_CMD_DATA,    // Wait 1 cycle for FIFO read data to settle; route to write/read path
    FETCH_DATA,       // Issue FIFO read enable for write data
    WAIT_WRITE_DATA,  // Wait 1 cycle for write data to settle before APB setup
    APB_SETUP,        // Assert PSEL, present address/data (APB setup phase)
    APB_ACCESS,       // Assert PENABLE; wait for PREADY (APB access phase)
    WRITE_RESP,       // Return write response (OKAY or SLVERR) to AXI
    READ_STORE,       // Push successful read beat into read-data FIFO
    ERROR_SINK_DATA,  // Drain remaining write beats on error (prevents bus stall)
    ERROR_READ_BURST  // Generate SLVERR + zero-data beats for erroneous reads
} state_t;

state_t state, next_state;

// =============================================================================
// Command & Data Registers (latched from FIFOs)
// =============================================================================
logic [ADDR_WIDTH-1:0]   cmd_addr_reg;
logic                    cmd_write_reg;
logic [ID_WIDTH-1:0]     cmd_id_reg;
logic [7:0]              cmd_len_reg;
logic [1:0]              cmd_burst_reg;
logic [2:0]              cmd_size_reg;
logic [2:0]              cmd_prot_reg;

logic [DATA_WIDTH-1:0]   wdata_reg;
logic [DATA_WIDTH/8-1:0] wstrb_reg;
logic                    wlast_reg;
logic [ID_WIDTH-1:0]     wdata_id_reg;

// =============================================================================
// Control Registers
// =============================================================================
logic [7:0] timeout_counter;
logic       error_flag;    // Combinational: set for one cycle on fault
logic       sticky_error;  // Sequential: latches any fault for the full burst
logic       addr_out_of_range;

// Burst beat counter; ADDR_INCR = bytes per beat for the configured data width
logic [7:0]         burst_cnt;
localparam ADDR_INCR = DATA_WIDTH / 8;

// =============================================================================
// Request FIFO Unpack
// FIFO word layout (LSB to MSB):
//   [2:0]   prot   (3b)
//   [5:3]   size   (3b)
//   [7:6]   burst  (2b)
//   [15:8]  len    (8b)
//   [15+ID_WIDTH:16]       id     (ID_WIDTH)
//   [16+ID_WIDTH]          write  (1b)
//   [16+ID_WIDTH+ADDR_WIDTH:17+ID_WIDTH]  addr (ADDR_WIDTH)
// =============================================================================
localparam REQ_PROT_LOW   = 0;
localparam REQ_PROT_HIGH  = REQ_PROT_LOW  + 3 - 1;
localparam REQ_SIZE_LOW   = REQ_PROT_HIGH + 1;
localparam REQ_SIZE_HIGH  = REQ_SIZE_LOW  + 3 - 1;
localparam REQ_BURST_LOW  = REQ_SIZE_HIGH + 1;
localparam REQ_BURST_HIGH = REQ_BURST_LOW + 2 - 1;
localparam REQ_LEN_LOW    = REQ_BURST_HIGH + 1;
localparam REQ_LEN_HIGH   = REQ_LEN_LOW   + 8 - 1;
localparam REQ_ID_LOW     = REQ_LEN_HIGH  + 1;
localparam REQ_ID_HIGH    = REQ_ID_LOW    + ID_WIDTH - 1;
localparam REQ_WRITE_LOW  = REQ_ID_HIGH   + 1;
localparam REQ_WRITE_HIGH = REQ_WRITE_LOW + 1 - 1;
localparam REQ_ADDR_LOW   = REQ_WRITE_HIGH + 1;
localparam REQ_ADDR_HIGH  = REQ_ADDR_LOW  + ADDR_WIDTH - 1;

wire [2:0]          req_prot  = req_fifo_rd_data[REQ_PROT_HIGH  : REQ_PROT_LOW];
wire [2:0]          req_size  = req_fifo_rd_data[REQ_SIZE_HIGH  : REQ_SIZE_LOW];
wire [1:0]          req_burst = req_fifo_rd_data[REQ_BURST_HIGH : REQ_BURST_LOW];
wire [7:0]          req_len   = req_fifo_rd_data[REQ_LEN_HIGH   : REQ_LEN_LOW];
wire [ID_WIDTH-1:0] req_id    = req_fifo_rd_data[REQ_ID_HIGH    : REQ_ID_LOW];
wire                req_write = req_fifo_rd_data[REQ_WRITE_HIGH : REQ_WRITE_LOW];
wire [ADDR_WIDTH-1:0] req_addr = req_fifo_rd_data[REQ_ADDR_HIGH : REQ_ADDR_LOW];

// =============================================================================
// Write Data FIFO Unpack
// FIFO word layout (LSB to MSB):
//   [ID_WIDTH-1:0]                  id   (ID_WIDTH)
//   [ID_WIDTH]                      last (1b)
//   [ID_WIDTH+DATA_WIDTH/8:ID_WIDTH+1]  strb (DATA_WIDTH/8)
//   [top:...]                       data (DATA_WIDTH)
// =============================================================================
localparam WDATA_ID_LOW    = 0;
localparam WDATA_ID_HIGH   = WDATA_ID_LOW   + ID_WIDTH    - 1;
localparam WDATA_LAST_LOW  = WDATA_ID_HIGH  + 1;
localparam WDATA_LAST_HIGH = WDATA_LAST_LOW + 1 - 1;
localparam WDATA_STRB_LOW  = WDATA_LAST_HIGH + 1;
localparam WDATA_STRB_HIGH = WDATA_STRB_LOW  + DATA_WIDTH/8 - 1;
localparam WDATA_DATA_LOW  = WDATA_STRB_HIGH + 1;
localparam WDATA_DATA_HIGH = WDATA_DATA_LOW  + DATA_WIDTH  - 1;

wire [ID_WIDTH-1:0]     wfifo_id   = wdata_fifo_rd_data[WDATA_ID_HIGH   : WDATA_ID_LOW];
wire                    wfifo_last = wdata_fifo_rd_data[WDATA_LAST_HIGH : WDATA_LAST_LOW];
wire [DATA_WIDTH/8-1:0] wfifo_strb = wdata_fifo_rd_data[WDATA_STRB_HIGH : WDATA_STRB_LOW];
wire [DATA_WIDTH-1:0]   wfifo_data = wdata_fifo_rd_data[WDATA_DATA_HIGH : WDATA_DATA_LOW];

// =============================================================================
// Helper: Last Read Beat Detection
// =============================================================================
logic is_last_read_beat;
assign is_last_read_beat = (burst_cnt == cmd_len_reg);

// =============================================================================
// Combinational Next-State & Output Logic
// =============================================================================
always_comb begin
    // --- Default outputs (avoids unintended latches) ---
    req_fifo_rd_en     = 0;
    wdata_fifo_rd_en   = 0;
    rdata_fifo_wr_en   = 0;
    rdata_fifo_wr_data = '0;
    apb_psel           = 0;
    apb_penable        = 0;
    apb_pwrite         = 0;
    apb_paddr          = '0;
    apb_pwdata         = '0;
    apb_pstrb          = '0;
    apb_pprot          = '0;
    bresp_valid        = 0;
    bresp_id           = '0;
    bresp_resp         = 2'b00;
    error_flag         = 0;
    addr_out_of_range  = (cmd_addr_reg < ADDR_BASE) ||
                         (cmd_addr_reg >= (ADDR_BASE + ADDR_SIZE));
    next_state         = state;

    case (state)
        // -----------------------------------------------------------------
        IDLE: begin
            if (!req_fifo_empty)
                next_state = FETCH_CMD;
        end

        // -----------------------------------------------------------------
        FETCH_CMD: begin
            req_fifo_rd_en = 1;        // Pop command from FIFO
            next_state     = WAIT_CMD_DATA;
        end

        // -----------------------------------------------------------------
        WAIT_CMD_DATA: begin
            // 1-cycle latency for FIFO read data to propagate into registers
            if (cmd_write_reg) begin
                if (addr_out_of_range)
                    next_state = ERROR_SINK_DATA;
                else if (!wdata_fifo_empty)
                    next_state = FETCH_DATA;
                // else: stall until write data is available
            end else begin
                next_state = addr_out_of_range ? ERROR_READ_BURST : APB_SETUP;
            end
        end

        // -----------------------------------------------------------------
        FETCH_DATA: begin
            wdata_fifo_rd_en = 1;      // Pop write beat from FIFO
            next_state       = WAIT_WRITE_DATA;
        end

        // -----------------------------------------------------------------
        WAIT_WRITE_DATA: begin
            // 1-cycle latency for write data to settle into registers
            next_state = (sticky_error || addr_out_of_range) ? ERROR_SINK_DATA
                                                              : APB_SETUP;
        end

        // -----------------------------------------------------------------
        APB_SETUP: begin
            apb_psel    = 1;
            apb_penable = 0;
            apb_paddr   = cmd_addr_reg - ADDR_BASE;
            apb_pwrite  = cmd_write_reg;
            apb_pwdata  = cmd_write_reg ? wdata_reg : '0;
            apb_pstrb   = cmd_write_reg ? wstrb_reg : '0;
            apb_pprot   = cmd_prot_reg;
            next_state  = APB_ACCESS;
        end

        // -----------------------------------------------------------------
        APB_ACCESS: begin
            apb_psel    = 1;
            apb_penable = 1;
            apb_paddr   = cmd_addr_reg - ADDR_BASE;
            apb_pwrite  = cmd_write_reg;
            apb_pwdata  = cmd_write_reg ? wdata_reg : '0;
            apb_pstrb   = cmd_write_reg ? wstrb_reg : '0;
            apb_pprot   = cmd_prot_reg;

            if (timeout_counter == 8'hFF) begin
                // APB timeout: abort remaining burst gracefully
                error_flag = 1;
                next_state = cmd_write_reg ? ERROR_SINK_DATA : ERROR_READ_BURST;
            end else if (apb_pready) begin
                if (apb_pslverr) begin
                    // Peripheral error: abort remaining burst gracefully
                    error_flag = 1;
                    next_state = cmd_write_reg ? ERROR_SINK_DATA : ERROR_READ_BURST;
                end else if (cmd_write_reg) begin
                    // Write path: continue or finish burst
                    if (burst_cnt < cmd_len_reg)
                        next_state = wdata_fifo_empty ? WAIT_CMD_DATA : FETCH_DATA;
                    else
                        next_state = WRITE_RESP;
                end else begin
                    // Read path: store successful beat
                    next_state = READ_STORE;
                end
            end
        end

        // -----------------------------------------------------------------
        WRITE_RESP: begin
            bresp_valid = 1;
            bresp_id    = cmd_id_reg;
            bresp_resp  = sticky_error ? 2'b10 : 2'b00; // SLVERR or OKAY
            if (bresp_ready)
                next_state = IDLE;
        end

        // -----------------------------------------------------------------
        READ_STORE: begin
            if (!rdata_fifo_full) begin
                rdata_fifo_wr_en   = 1;
                // FIFO word: {data, id, last, resp=OKAY}
                rdata_fifo_wr_data = {apb_prdata, cmd_id_reg, is_last_read_beat, 2'b00};
                next_state = (burst_cnt < cmd_len_reg) ? APB_SETUP : IDLE;
            end
            // else: stall until FIFO has space
        end

        // -----------------------------------------------------------------
        ERROR_SINK_DATA: begin
            // Drain remaining write beats so the write-data FIFO doesn't stall
            if (burst_cnt <= cmd_len_reg) begin
                next_state = wdata_fifo_empty ? ERROR_SINK_DATA : FETCH_DATA;
            end else begin
                next_state = WRITE_RESP; // All beats absorbed; send SLVERR response
            end
        end

        // -----------------------------------------------------------------
        ERROR_READ_BURST: begin
            // Return zero-data + SLVERR for every expected read beat
            if (!rdata_fifo_full) begin
                rdata_fifo_wr_en   = 1;
                // FIFO word: {data=0, id, last, resp=SLVERR}
                rdata_fifo_wr_data = {'0, cmd_id_reg, is_last_read_beat, 2'b10};
                next_state = (burst_cnt < cmd_len_reg) ? ERROR_READ_BURST : IDLE;
            end
            // else: stall until FIFO has space
        end

        // -----------------------------------------------------------------
        default: next_state = IDLE;
    endcase
end

// =============================================================================
// Sequential Register Update
// =============================================================================
always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        state           <= IDLE;
        timeout_counter <= '0;
        cmd_addr_reg    <= '0;
        cmd_write_reg   <= '0;
        cmd_id_reg      <= '0;
        cmd_len_reg     <= '0;
        cmd_burst_reg   <= '0;
        cmd_size_reg    <= '0;
        cmd_prot_reg    <= '0;
        wdata_reg       <= '0;
        wstrb_reg       <= '0;
        wlast_reg       <= '0;
        wdata_id_reg    <= '0;
        burst_cnt       <= '0;
        sticky_error    <= '0;
    end else begin
        state <= next_state;

        // Reset burst state at the start of every new command
        if (state == IDLE && !req_fifo_empty) begin
            burst_cnt    <= '0;
            sticky_error <= '0;
        end

        // Advance burst counter and address after each successful APB beat
        if (state == APB_ACCESS && apb_pready && !apb_pslverr) begin
            if (burst_cnt < cmd_len_reg) begin
                burst_cnt <= burst_cnt + 1;
                if (cmd_burst_reg == 2'b01)          // INCR burst only
                    cmd_addr_reg <= cmd_addr_reg + ADDR_INCR;
            end
        end else if (state == ERROR_READ_BURST && !rdata_fifo_full) begin
            if (burst_cnt < cmd_len_reg) begin
                burst_cnt <= burst_cnt + 1;
                if (cmd_burst_reg == 2'b01)
                    cmd_addr_reg <= cmd_addr_reg + ADDR_INCR;
            end
        end else if (state == ERROR_SINK_DATA && next_state == FETCH_DATA) begin
            // Count each discarded write beat
            if (burst_cnt <= cmd_len_reg)
                burst_cnt <= burst_cnt + 1;
        end

        // Latch any fault into sticky_error for the whole burst
        if ((state == WAIT_CMD_DATA && addr_out_of_range) || error_flag)
            sticky_error <= 1'b1;

        // APB timeout counter: increment while waiting for PREADY
        if (state == APB_ACCESS && !apb_pready)
            timeout_counter <= timeout_counter + 1;
        else
            timeout_counter <= '0;

        // Latch command fields from request FIFO (one cycle after rd_en)
        if (req_fifo_rd_en) begin
            cmd_addr_reg  <= req_addr;
            cmd_write_reg <= req_write;
            cmd_id_reg    <= req_id;
            cmd_len_reg   <= req_len;
            cmd_burst_reg <= req_burst;
            cmd_size_reg  <= req_size;
            cmd_prot_reg  <= req_prot;
        end

        // Latch write data fields from write-data FIFO (one cycle after rd_en)
        if (wdata_fifo_rd_en) begin
            wdata_reg    <= wfifo_data;
            wstrb_reg    <= wfifo_strb;
            wlast_reg    <= wfifo_last;
            wdata_id_reg <= wfifo_id;
        end
    end
end

endmodule