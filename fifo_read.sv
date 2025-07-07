// roi toi fifo
module fifo_read#(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32,
    parameter DEPTH = 16
)(
    input  logic clk,
    input  logic reset,
    input  logic rd_en,                      // read enable
    output logic [DATA_WIDTH-1:0] rd_data,   // read data
    output logic full,                       // full (không dùng nếu chỉ đọc)
    output logic empty,                      // empty
    output logic [ADDR_WIDTH-1:0] wr_ptr,    // write pointer (không thay đổi ở đây)
    output logic [ADDR_WIDTH-1:0] rd_ptr,    // read pointer
    output logic [ADDR_WIDTH-1:0] count,
    output logic [ADDR_WIDTH-1:0] level
);

    // FIFO MEMORY (giả sử đã có dữ liệu)
    logic [DATA_WIDTH-1:0] mem [DEPTH-1:0];
    logic [ADDR_WIDTH-1:0] rd_ptr_reg;
    logic [ADDR_WIDTH-1:0] count_reg;

    // Không có logic ghi, wr_ptr cố định
    assign wr_ptr = '0; // hoặc giá trị bạn mong muốn

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            rd_ptr_reg <= 0;
            count_reg  <= DEPTH; // Nếu mem đã đầy sẵn
        end else begin
            if (rd_en && !empty) begin
                rd_ptr_reg <= rd_ptr_reg + 1;
                count_reg  <= count_reg - 1;
            end
        end
    end

    assign rd_data = mem[rd_ptr_reg];
    assign rd_ptr  = rd_ptr_reg;
    assign count   = count_reg;
    assign level   = count_reg;
    assign full    = (count_reg == DEPTH); // luôn full nếu không có ghi
    assign empty   = (count_reg == 0);

endmodule