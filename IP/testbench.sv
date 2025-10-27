`timescale 1ns / 1ps

module testbench;

    // Parameters
    parameter DATA_WIDTH = 32;
    parameter ADDR_WIDTH = 32;
    parameter ID_WIDTH = 4;
    parameter CLK_PERIOD = 10; // 100MHz
    parameter ADDR_BASE = 32'h1000_0000;

    // Clock and reset
    logic clk;
    logic reset;

    // AXI4 Slave Interface signals
    logic awvalid, awready;
    logic [ADDR_WIDTH-1:0] awaddr;
    logic [2:0] awsize;
    logic [7:0] awlen;
    logic [1:0] awburst;
    logic [ID_WIDTH-1:0] awid;
    logic [2:0] awprot;
    logic awlock;
    logic [3:0] awcache, awqos, awregion;
    logic arvalid, arready;
    logic [ADDR_WIDTH-1:0] araddr;
    logic [2:0] arsize;
    logic [7:0] arlen;
    logic [1:0] arburst;
    logic [ID_WIDTH-1:0] arid;
    logic [2:0] arprot;
    logic arlock;
    logic [3:0] arcache, arqos, arregion;

    logic wvalid, wready;
    logic [DATA_WIDTH-1:0] wdata;
    logic [DATA_WIDTH/8-1:0] wstrb;
    logic wlast;
    logic [ID_WIDTH-1:0] wid;

    logic rvalid, rready;
    logic [DATA_WIDTH-1:0] rdata;
    logic rlast;
    logic [1:0] rresp;
    logic [ID_WIDTH-1:0] rid;
    logic bvalid, bready;
    logic [1:0] bresp;
    logic [ID_WIDTH-1:0] bid;

    // APB4 Master Interface signals
    logic [ADDR_WIDTH-1:0] paddr;
    logic pwrite;
    logic psel;
    logic penable;
    logic [DATA_WIDTH-1:0] pwdata;
    logic [DATA_WIDTH/8-1:0] pstrb; // Sửa độ rộng
    logic [2:0] pprot;
    logic [DATA_WIDTH-1:0] prdata;
    logic pready;
    logic pslverr;

    // Other signals
    logic irq;
    logic debug_req;
    logic bridge_active;
    logic [31:0] error_count_out;
    logic [31:0] transaction_count_out;
    
    // Test variables
    integer test_count = 0;
    integer error_count = 0;
    logic [DATA_WIDTH-1:0] read_data;
    
    // Clock generation
    initial begin
        clk = 0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end

    // Sửa: Khởi tạo axi_slave (DUT cấp cao nhất)
    axi_slave #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .ID_WIDTH(ID_WIDTH),
        .ADDR_BASE(ADDR_BASE)
    ) dut (
        .clk(clk),
        .reset(reset),
        
        // AXI4 Slave Interface
        .awvalid(awvalid), .awready(awready), .awaddr(awaddr), .awsize(awsize),
        .awlen(awlen), .awburst(awburst), .awid(awid), .awprot(awprot),
        .awlock(awlock), .awcache(awcache), .awqos(awqos), .awregion(awregion),
        
        .arvalid(arvalid), .arready(arready), .araddr(araddr), .arsize(arsize),
        .arlen(arlen), .arburst(arburst), .arid(arid), .arprot(arprot),
        .arlock(arlock), .arcache(arcache), .arqos(arqos), .arregion(arregion),
        
        .wvalid(wvalid), .wready(wready), .wdata(wdata), .wstrb(wstrb),
        .wlast(wlast), .wid(wid),
        
        .rvalid(rvalid), .rready(rready), .rdata(rdata), .rlast(rlast),
        .rresp(rresp), .rid(rid),
        
        .bvalid(bvalid), .bready(bready), .bresp(bresp), .bid(bid),
        
        // APB4 Master Interface
        .paddr(paddr), .pwrite(pwrite), .psel(psel), .penable(penable),
        .pwdata(pwdata), .pstrb(pstrb), .pprot(pprot),
        .prdata(prdata), .pready(pready), .pslverr(pslverr),
        
        // Other
        .irq(irq), 
        .debug_req(debug_req),
        .bridge_active(bridge_active),
        .error_count(error_count_out),
        .transaction_count(transaction_count_out)
    );
    
    // APB Slave Model (Simple memory model)
    logic [DATA_WIDTH-1:0] apb_memory [1024]; // 4KB memory
    
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            pready <= 1'b0;
            pslverr <= 1'b0;
            prdata <= '0;
        end else begin
            pready <= 1'b0;
            pslverr <= 1'b0;
            
            if (psel && penable) begin
                pready <= 1'b1;
                if (pwrite) begin
                    // Write operation
                    apb_memory[paddr[11:2]] <= pwdata; // Word aligned
                    $display("APB Write: Addr=0x%08x, Data=0x%08x", paddr, pwdata);
                end else begin
                    // Read operation
                    prdata <= apb_memory[paddr[11:2]];
                    $display("APB Read: Addr=0x%08x, Data=0x%08x", paddr, apb_memory[paddr[11:2]]);
                end
            end
        end
    end

    // Initialize signals
    task init_signals();
        awvalid = 0; awaddr = 0; awsize = 0; awlen = 0; awburst = 0;
        awid = 0;
        awprot = 0; awlock = 0; awcache = 0; awqos = 0; awregion = 0;
        
        arvalid = 0; araddr = 0; arsize = 0; arlen = 0; arburst = 0;
        arid = 0; arprot = 0;
        arlock = 0; arcache = 0; arqos = 0; arregion = 0;
        
        wvalid = 0; wdata = 0;
        wstrb = 0; wlast = 0; wid = 0;
        rready = 1; bready = 1;
        debug_req = 0;
    endtask

    // Reset task
    task reset_dut();
        reset = 1;
        init_signals();
        repeat(5) @(posedge clk);
        reset = 0;
        repeat(2) @(posedge clk);
        $display("Reset completed");
    endtask

    // AXI Write task
    task axi_write(
        input [ADDR_WIDTH-1:0] addr,
        input [DATA_WIDTH-1:0] data,
        input [ID_WIDTH-1:0] id
    );
        begin
            test_count++;
            $display("Test %0d: AXI Write - Addr=0x%08x, Data=0x%08x, ID=%0d", 
                     test_count, addr, data, id);
            fork
                // Write Address Channel
                begin
                    awvalid = 1;
                    awaddr = addr;
                    awsize = 3'b010; // 32-bit
                    awlen = 8'h00; // Single beat
                    awburst = 2'b01; // INCR
                    awid = id;
                    awprot = 3'b000;
                    awlock = 0;
                    awcache = 4'h0;
                    awqos = 4'h0;
                    awregion = 4'h0;
                    
                    wait(awready);
                    @(posedge clk);
                    awvalid = 0;
                end
                
                // Write Data Channel
                begin
                    wvalid = 1;
                    wdata = data;
                    wstrb = 4'hF; // All bytes
                    wlast = 1;
                    wid = id;
                    
                    wait(wready);
                    @(posedge clk);
                    wvalid = 0;
                    wlast = 0;
                end
            join
            
            // Wait for Write Response
            wait(bvalid);
            @(posedge clk);
            
            if (bresp == 2'b00 && bid == id) begin
                $display("--> Write successful");
            end else begin
                $display("--> Write failed - bresp=%b, bid=%0d", bresp, bid);
                error_count++;
            end
        end
    endtask

    // AXI Read task
    task axi_read(
        input [ADDR_WIDTH-1:0] addr,
        input [ID_WIDTH-1:0] id,
        output [DATA_WIDTH-1:0] read_data_out
    );
        begin
            test_count++;
            $display("Test %0d: AXI Read - Addr=0x%08x, ID=%0d", test_count, addr, id);
            // Read Address Channel
            arvalid = 1;
            araddr = addr;
            arsize = 3'b010; // 32-bit
            arlen = 8'h00;
            arburst = 2'b01; // INCR
            arid = id;
            arprot = 3'b000;
            arlock = 0;
            arcache = 4'h0;
            arqos = 4'h0;
            arregion = 4'h0;
            
            wait(arready);
            @(posedge clk);
            arvalid = 0;
            
            // Wait for Read Data
            wait(rvalid);
            read_data_out = rdata;
            @(posedge clk);
            
            if (rresp == 2'b00 && rid == id && rlast) begin
                $display("--> Read successful - Data=0x%08x", read_data_out);
            end else begin
                $display("--> Read failed - rresp=%b, rid=%0d, rlast=%b", rresp, rid, rlast);
                error_count++;
            end
        end
    endtask


    initial begin
        $display("Starting AXI to APB Bridge Testbench");
        // Initialize
        reset_dut();
        
        // Sửa: Sử dụng địa chỉ trong phạm vi ADDR_BASE
        // Test 1: Simple Write
        axi_write(ADDR_BASE + 32'h0004, 32'hDEADBEEF, 4'h1); // Ghi vào apb_memory[1]
        repeat(10) @(posedge clk);
        
        // Test 2: Simple Read (same address)
        axi_read(ADDR_BASE + 32'h0004, 4'h2, read_data); // Đọc từ apb_memory[1]
        repeat(10) @(posedge clk);
        
        // Verify read data matches written data
        if (read_data == 32'hDEADBEEF) begin
            $display("--> Read-after-write verification passed");
        end else begin
            $display("--> Read-after-write verification failed - Expected: 0x%08x, Got: 0x%08x", 
                     32'hDEADBEEF, read_data);
            error_count++;
        end
        
        // Test 3: Multiple writes to different addresses
        axi_write(ADDR_BASE + 32'h0008, 32'h12345678, 4'h3); // Ghi vào apb_memory[2]
        repeat(10) @(posedge clk);
        axi_write(ADDR_BASE + 32'h000C, 32'h87654321, 4'h4); // Ghi vào apb_memory[3]
        repeat(10) @(posedge clk);
        
        // Test 4: Read back the multiple writes
        axi_read(ADDR_BASE + 32'h0008, 4'h5, read_data); // Đọc từ apb_memory[2]
        repeat(10) @(posedge clk);
        axi_read(ADDR_BASE + 32'h000C, 4'h6, read_data); // Đọc từ apb_memory[3]
        repeat(10) @(posedge clk);
        
        // Test 5: Viết vào địa chỉ không hợp lệ (ngoài phạm vi)
        axi_write(32'hBADC0FFE, 32'hBADDATA, 4'h7);
        repeat(10) @(posedge clk);
        
        // Test 6: Đọc từ địa chỉ không hợp lệ (ngoài phạm vi)
        axi_read(32'hBADC0FFE, 4'h8, read_data);
        repeat(10) @(posedge clk);

        // Test completion
        repeat(20) @(posedge clk);
        $display("\n=== Test Summary ===");
        $display("Total tests: %0d", test_count);
        $display("Errors (Testbench): %0d", error_count);
        $display("Errors (DUT): %0d", error_count_out);
        
        if (error_count == 0 && error_count_out == 2) begin 
            $display("--> All tests PASSED!");
        end else begin
            $display("--> Test FAILED!");
        end
        
        $finish;
    end

    // Timeout watchdog
    initial begin
        #100000; // 100us timeout
        $display("ERROR: Testbench timeout!");
        $finish;
    end

endmodule