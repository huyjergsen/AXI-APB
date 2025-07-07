module core(
    //clock and reset
    clk,
    reset, 
        //Address write
        awvalid,
        awready,
        awaddr,
        awsize,
        awlen,
        awburst,
        awid,
        awprot,
        //Address read
        arvalid,
        arready,
        araddr,
        arsize,
        arlen,
        arburst,
        arid,
        arprot,
        //Data write    
        wvalid,
        wready,
        wdata,
        wstrb,
        wlast,
        wid,
        //Data read
        rvalid,
        rready,
        rlast,
        rresp,
        rid,
        rdata,
        //Response
        bvalid,
        bready,
        bresp,
        bid,
        //Interrupt
        irq,
        //Debug
        debug_req,
        //apb signals
        paddr,
        pwrite,
        psel,
        penable,
        pclk,
        presetn,
        //apb slave signals
        psel_slave,
        penable_slave,
        pwdata,
        prdata,
        pready,
        pslverr,
        

