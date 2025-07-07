module fifo_request#(
    parameter DATA_WIDTH = 32,
    parameter NUM_FIFO = 2 // Số lượng FIFO có thể chọn
)(
    input  logic clk,
    input  logic reset,
    input  logic req,                              // yêu cầu lấy dữ liệu
    input  logic [$clog2(NUM_FIFO)-1:0] fifo_sel,  // chọn FIFO
    input  logic [NUM_FIFO-1:0] fifo_empty,        // trạng thái từng FIFO
    input  logic [DATA_WIDTH-1:0] fifo_data [NUM_FIFO], // dữ liệu từ từng FIFO
    output logic [NUM_FIFO-1:0] fifo_rd_en,        // tín hiệu đọc từng FIFO
    output logic [DATA_WIDTH-1:0] req_data,        // dữ liệu xuất ra
    output logic valid,                            // dữ liệu hợp lệ
    output logic busy,                             // FSM đang bận
    output logic error,                            // báo lỗi khi FIFO rỗng
    output logic [31:0] req_count                  // đếm số yêu cầu thành công
    // output logic ack,                           // Có thể bỏ nếu dùng valid
);

    typedef enum logic [1:0] {
        IDLE,
        REQ,
        WAIT_DATA,
        VALID
    } state_t;

    state_t state, next_state;
    logic [DATA_WIDTH-1:0] data_buf;
    logic [31:0] req_count_reg;

    // FSM chuyển trạng thái và đếm số yêu cầu
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
            data_buf <= '0;
            req_count_reg <= 0;
        end else begin
            state <= next_state;
            if (state == WAIT_DATA)
                data_buf <= fifo_data[fifo_sel];
            if (state == VALID && next_state == IDLE)
                req_count_reg <= req_count_reg + 1;
        end
    end

    always_comb begin
        next_state = state;
        fifo_rd_en = '0;
        req_data = '0;
        valid = 0;
        busy = 1;
        error = 0;
        case (state)
            IDLE: begin
                busy = 0;
                if (req) begin
                    if (!fifo_empty[fifo_sel]) begin
                        next_state = REQ;
                    end else begin
                        error = 1;
                        next_state = IDLE;
                    end
                end
            end
            REQ: begin
                fifo_rd_en[fifo_sel] = 1;
                next_state = WAIT_DATA;
            end
            WAIT_DATA: begin
                next_state = VALID;
            end
            VALID: begin
                valid = 1;
                req_data = data_buf;
                if (!req)
                    next_state = IDLE;
            end
        endcase
    end

    assign req_count = req_count_reg;

endmodule