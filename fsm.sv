module fsm#(
    logic [DATA_WIDTH-1:0] apb_paddr,
    logic [ADDR_WIDTH-1:0] apb_pwdata, apb_prdata,
    logic [DATA_WIDTH-1:0] axi_wdata, axi_rdata,
    logic [1:0] axi_cmd_type,  // 2 bit cho loai lenh (read/write)
    //
    logic fifo_req_rd_en,      // Đọc FIFO Request
    logic fifo_write_rd_en,    // Đọc FIFO Write
    logic fifo_read_wr_en,
    logic apb_psel,
    logic apb_penable,  
    logic apb_pwrite,
    logic apb_ready,
    logic apb_slverr,
    logic axi_resp_valid,
    logic error_flag,
)

typedef enum logic [3:0]{
    IDLE,
    FETCH_CMD,
    FETCH_DATA,
    APB_SETUP,
    APB_ACCESS,
    WRITE_RESP,
    READ_STORE,
    READ_RESP,
    ERROR
} state_t;

state_t state, next_state;

always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        state <= IDLE;
    end else begin
        state <= next_state;
    end
end

always_comb begin
    fifo_req_rd_en = 0;
    fifo_write_rd_en = 0;
    fifo_read_wr_en = 0;
    apb_psel = 0;   
    apb_penable = 0;
    apb_pwrite = 0;
    axi_resp_valid = 0;
    error_flag = 0;
    next_state = state;

    case(state)
        IDLE: begin
            if (fifo_read_wr_en) begin
                next_state = FETCH_CMD;
            end else if (fifo_write_rd_en) begin
                next_state = FETCH_DATA;
            end else if (fifo_req_rd_en) begin
                next_state = APB_SETUP;
            end else begin
                next_state = IDLE;
            end

        FETCH_CMD: begin
            if(axi_cmd_type == 2'b01) begin // read
                if(!fifo_req_empty) begin
                    fifo_req_rd_en = 1;
                    next_state = FETCH_DATA;
                end else begin
                    error_flag = 1;
                    next_state = ERROR;
                end
            end else if(axi_cmd_type == 2'b10) begin // write
                if(!fifo_write_empty) begin
                    fifo_write_rd_en = 1;
                    next_state = APB_SETUP;
                end else begin
                    error_flag = 1;
                    next_state = ERROR;
                end
            end else begin
                error_flag = 1;
                next_state = ERROR;
            end

        APB_SETUP: begin
            apb_psel = 1;
            apb_penable = 0;
            apb_paddr = cmd_addr;
            apb_pwrite = (axi_cmd_type == 2'b10); // 1: write, 0: read
            apb_pwdata = (axi_cmd_type == 2'b10) ? axi_wdata : '0;
            next_state = APB_ACCESS;
        end

        APB_ACCESS: begin
            apb_penable = 1;
            if (apb_ready) begin
                if(apb_slverr) begin
                    error_flag = 1;
                    next_state = ERROR;
                end else if (apb_pwrite) begin
                    next_state = WRITE_RESP;
                end else begin
                    next_state = READ_RESP;
                end
            end else begin
                next_state = APB_ACCESS; // Chờ APB sẵn sàng
            end
        end

        WRITE_RESP: begin
            axi_resp_valid = 1; // write success
            if(!fifo_write_empty) begin
                fifo_write_rd_en = 1; // Đọc FIFO Write để lấy lệnh tiếp theo
                next_state = FETCH_CMD; // Quay lại FETCH_CMD để xử lý lệnh tiếp theo
            end else begin
                next_state = IDLE; // Không còn lệnh nào, quay về IDLE
            end
        end

        READ_STORE: begin
            axi_rdata = apb_prdata; // Lưu dữ liệu đọc từ APB
            axi_resp_valid = 1; // Đánh dấu phản hồi đã sẵn sàng
            if(!fifo_read_empty) begin
                fifo_read_wr_en = 1; // Ghi dữ liệu vào FIFO Read
                next_state = READ_RESP; // Chuyển sang trạng thái đọc phản hồi
            end else begin
                next_state = IDLE; // Không còn lệnh nào, quay về IDLE
            end
        end

        READ_RESP: begin

        end