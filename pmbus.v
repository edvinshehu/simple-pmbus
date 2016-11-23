`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    19:07:21 10/21/2016 
// Design Name: 
// Module Name:    pmbus 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module pmbus(
    input EN,
    input rst,
    input clk,
    input i2cclk,
    input i2cdata,
    input [7:0] vout,
    input [7:0] iout,
    input [7:0] tmp,
    input [7:0] vin,
    input unitoff,
	 input voutov,
	 input ioutoc,
	 input vinuv,
	 input tmpov,
	 input sysalert,
	 output i2cdataout,
	 output smbalout,
    output [7:0] vid
    );

//outputs of cmm that go to prreg
wire [5:0] i2cclkloout;
wire dbusyout;
wire [7:0] cmdout;
wire [7:0] ldbout;
wire [7:0] hdbout;
wire arbtrout;
wire smbalclrout;
wire stopout;
wire startout;
wire rwbit;
wire [7:0] addrout;
wire [6:0] daddrout;
wire rstart;

//outputs of prreg that go to cmm
wire cmdv;
wire ldbv;
wire hdbv;
wire [7:0] ldbw;
wire [7:0] hdbw;
wire [7:0] statusout;
wire cmlbitout;

cmm cmm_0 (
	.EN(EN),
	.rst(rst),
	.clk(clk),
	.i2cclk(i2cclk),
	.i2cdata(i2cdata),
	.cmdvalid(cmdv),
	.ldbvalid(ldbv),
	.hdbvalid(hdbv),
	.ldbw(ldbw),
	.hdbw(hdbw),
	.smbal(smbalout),
	.cmlbit(cmlbitout),
	.i2cdataout(i2cdataout),
	.i2cclkloout(i2cclkloout),
	.dbusyout(dbusyout),
	.cmdout(cmdout),
	.ldbout(ldbout),
	.hdbout(hdbout),
	.startout(startout),
	.stopout(stopout),
	.rwbit(rwbit),
	.arbtrout(arbtrout),
	.smbalclrout(smbalclrout),
	.addrout(addrout),
	.daddrout(daddrout),
	.rstartout(rstart)
	);
	

prreg prreg_0 (
	.EN(EN),
	.rst(rst),
	.clk(clk),
	.cmd(cmdout),
	.ldb(ldbout),
	.hdb(hdbout),
	.rwbit(rwbit),
	.dbusy(dbusyout),
	.start(startout),
	.stop(stopout),
	.count(i2cclkloout),
	.vout(vout),
	.iout(iout),
	.tmp(tmp),
	.vin(vin),
	.unitoff(unitoff),
	.voutov(voutov),
	.ioutoc(ioutoc),
	.vinuv(vinuv),
	.tmpov(tmpov),
	.smbalclr(smbalclrout),
	.addr(addrout),
	.arbtr(arbtrout),
	.sysalert(sysalert),
	.rstart(rstart),
	.cmdv(cmdv),
	.ldbv(ldbv),
	.hdbv(hdbv),
	.ldbw(ldbw),
	.hdbw(hdbw),
	.vid(vid),
	.statusout(statusout),
	.smbalout(smbalout),
	.cmlbitout(cmlbitout)
	);

endmodule
