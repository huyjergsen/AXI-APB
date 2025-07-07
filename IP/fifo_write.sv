// roi toi fifo
module fifo_write#(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32,
    parameter DEPTH = 16
)(
    input logic clk,
    input logic reset,
    input logic wr_en,                  // write enable
    input logic [DATA_WIDTH-1:0] wr_data, // write data
    input logic rd_en,                  // read enable
    output logic [DATA_WIDTH-1:0] rd_data, // read data
    output logic full,                   // full
    output logic empty,                  // empty
    output logic [ADDR_WIDTH-1:0] wr_ptr, // write pointer
    output logic [ADDR_WIDTH-1:0] rd_ptr, // read pointer
    output logic [ADDR_WIDTH-1:0] count,
    output logic [ADDR_WIDTH-1:0] level
);

    // FIFO MEMORY
    logic [DATA_WIDTH-1:0] mem [DEPTH-1:0]; 
    // READ - WRITE POINTER
    logic [ADDR_WIDTH-1:0] wr_ptr_reg, rd_ptr_reg;
    logic [ADDR_WIDTH-1:0] count_reg;

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            wr_ptr_reg <= 0;
            rd_ptr_reg <= 0;
            count_reg  <= 0;
        end else begin
            // Write logic
            if (wr_en && !full) begin
                mem[wr_ptr_reg] <= wr_data;
                wr_ptr_reg <= wr_ptr_reg + 1;
            end
            // Read logic
            if (rd_en && !empty) begin
                rd_ptr_reg <= rd_ptr_reg + 1;
            end
            // Count logic
            case ({wr_en && !full, rd_en && !empty})
                2'b10: count_reg <= count_reg + 1; // Only write
                2'b01: count_reg <= count_reg - 1; // Only read
                default: count_reg <= count_reg;   // No change or both
            endcase
        end
    end

    // Xuất dữ liệu đọc
    assign rd_data = mem[rd_ptr_reg];

    // Báo trạng thái
    assign full  = (count_reg == DEPTH);
    assign empty = (count_reg == 0);

    // Xuất con trỏ và mức sử dụng
    assign wr_ptr = wr_ptr_reg;
    assign rd_ptr = rd_ptr_reg;
    assign count  = count_reg;
    assign level  = count_reg;

endmodule
