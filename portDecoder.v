/********************************************************************************
*   File:  portDecoder.v    TFOX    Ver 0.1     Oct.12, 2022                    *
*           TFOX, N4TLF January 31, 2023   You are free to use it               *
*           however you like.  No warranty expressed or implied                 *
*       I/O port decoding to create active HIGH "chip" or device selects        *
********************************************************************************/

module portDecoder
    (
//    input           clock,
    input [7:0]     address,
    input           iowrite,
    input           ioread,
    //
    output          outPortFF_cs,
    output          inPortCon_cs,
    output          outFbarLEDs_cs,
    output          inFbarLEDs_cs,
    output          outMiscCtl_cs,
    output          inIOBYTE_cs,
    output          outRAMA16_cs 
    );

    assign outPortFF_cs = (address[7:0] == 8'b11111111) && iowrite;
    assign inPortCon_cs = (address[7:1] == 7'b0000000) && ioread;
    assign outFbarLEDs_cs = (address[7:0] == 8'b00000110) && iowrite; // Port 06 write
    assign inFbarLEDs_cs = (address[7:0] == 8'b00000110) && ioread;  // port 06 read
    assign outMiscCtl_cs = (address[7:0] == 8'b00000111) && iowrite; // Port 07 out
    assign inIOBYTE_cs = (address[7:0] == 8'b00110110) && ioread;    // IN = IOBYTE(switches)
    assign outRAMA16_cs = (address[7:0] == 8'b00110110) && iowrite;  // Out 36 D0 = RAM A16

    endmodule
