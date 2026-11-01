`timescale 1ns/1ps
`default_nettype none
module tb_et_integrity;
  localparam AW=6,BINS=64,FRAME=16384;
  reg clk=0;always #5 clk=~clk;
  reg rst=0,mode=0,sv=0,locked=0,ref_ok=0,commit=0,freeze=0,hold=0;
  reg signed [11:0] x0=0,x1=0;
  reg [31:0] ph=0,fcw=66076420;
  reg signed [31:0] residual=0;
  wire pulse;wire [15:0] fid;
  wire [6:0] filled,ready;
  wire [31:0] wraps,hits,epoch;
  wire sl,sq,valid;
  reg [9:0] addr=0;
  wire signed [11:0] y0,y1;wire [15:0] n,dbg;
  et_display #(.ET_FRAME_CLKS(FRAME)) dut(
    .clk(clk),.rst_n(rst),.mode(mode),.sample_valid(sv),.ch0(x0),.ch1(x1),
    .nco_phase(ph),.nco_fcw(fcw),.afc_locked(locked),.reference_ok(ref_ok),
    .residual0_fcw(residual),.residual1_fcw(residual),.cfg_commit(commit),
    .freeze(freeze),.hold(hold),.frame_pulse(pulse),.frame_id(fid),
    .filled(filled),.ready_bins(ready),.wraps(wraps),.hits(hits),.epoch_id(epoch),
    .snapshot_locked(sl),.snapshot_reference_ok(sq),.rd_addr(addr),
    .rd_ch0(y0),.rd_ch1(y1),.rd_valid(valid),.rd_hits(n),.dbg_cnt(dbg));
  integer tests=0,amp=4,kind=0,noise=0,idx=0;
  reg [31:0] p=0,lfsr=32'hace12345;
  real physical_phase=0.173,signal_hz=1000000.0;
  integer v,j,k,good,worst,delta,old_hits,old_epoch;
  real energy,ground;
  reg signed [11:0] snap_y[0:63];
  reg snap_v[0:63];reg [15:0] snap_n[0:63];

  task reset_case;
    begin
      @(negedge clk);rst=0;mode=0;sv=0;locked=0;ref_ok=0;hold=0;freeze=0;commit=0;
      repeat(4)@(negedge clk);
      p=32'h2c49ba5e;ph=p;physical_phase=0.173;idx=0;noise=0;
      residual=0;fcw=32'd66076420;signal_hz=1000000.0;lfsr=32'hace12345;
      rst=1;mode=1;locked=1;ref_ok=1;
    end
  endtask
  // The analog waveform is generated from a REAL physical clock ratio, not
  // from the NCO word driving the DUT. This is the independent physics oracle.
  task step_sample;
    begin
      @(negedge clk);
      case(kind)
        0:v=amp;
        1:v=(idx%2)?-amp:amp;
        default:v=(physical_phase<0.5)?amp:-amp;
      endcase
      if(noise!=0)begin
        lfsr={lfsr[30:0],lfsr[31]^lfsr[21]^lfsr[1]^lfsr[0]};
        v=v+($unsigned(lfsr[15:0])%(2*noise+1))-noise;
      end
      x0=v;x1=(v== -2048)?2047:-v;sv=1;ph=p;
      @(posedge clk);#1;
      p=p+fcw;idx=idx+1;
      physical_phase=physical_phase+signal_hz/65000000.0;
      if(physical_phase>=1.0)physical_phase=physical_phase-1.0;
    end
  endtask
  task capture;
    integer limit;
    begin
      limit=0;
      while(!pulse && limit<70000)begin step_sample();limit=limit+1;end
      if(!pulse)$fatal(1,"No diagnostic snapshot");
      hold=1;
    end
  endtask
  task read_bin(input integer a);
    begin @(negedge clk);addr=a;repeat(2)@(negedge clk);#1;end
  endtask
  task check_constant(input integer value);
    begin
      reset_case();amp=value;kind=0;capture();
      if(ready!=64 || !sq)$fatal(1,"constant not ready %0d",ready);
      for(j=0;j<64;j=j+1)begin
        read_bin(j);
        if(!valid || y0!==value || y1!==((value== -2048)?2047:-value))
          $fatal(1,"weak/endpoint DC %0d bin %0d -> %0d/%0d",value,j,y0,y1);
      end
      tests=tests+1;$display("PASS constant %0d retained on both lanes",value);
    end
  endtask
  initial begin
    check_constant(1);check_constant(-1);check_constant(4);check_constant(-4);
    check_constant(8);check_constant(-8);check_constant(20);check_constant(-20);
    check_constant(2047);check_constant(-2048);

    reset_case();kind=0;amp=-4;
    for(k=0;k<4096;k=k+1)step_sample();
    amp=4;capture();
    for(j=0;j<64;j=j+1)begin
      read_bin(j);if(!valid || y0!=4 || y1!= -4)$fatal(1,"weak polarity step got %0d/%0d",y0,y1);
    end
    tests=tests+1;$display("PASS weak -4 to +4 step without warm-start/reset");

    // A 921600-baud ET frame can outlast FRAME. The reader freezes only the
    // snapshot; it must never stop live acquisition or produce invalid-only
    // frames forever. This assertion rejects the initial candidate.
    reset_case();kind=0;amp=4;capture();old_hits=hits;
    amp=-4;
    for(k=0;k<40000;k=k+1)begin
      step_sample();
      if(pulse)$fatal(1,"snapshot overwritten while reader busy");
    end
    if(dut.live_hits<128 || dut.live_ready==0)$fatal(1,"reader starved live acquisition");
    read_bin(10);if(y0!=4 || hits!=old_hits)$fatal(1,"reader snapshot changed");
    hold=0;capture();
    if(ready!=64)$fatal(1,"after long reader hold frame not ready");
    read_bin(10);if(y0!= -4)$fatal(1,"next frame failed to refresh");
    tests=tests+1;$display("PASS sustained UART busy: live acquisition continues, snapshot atomic, next frame ready");

    reset_case();kind=1;amp=4;capture();
    for(j=0;j<64;j=j+1)begin
      read_bin(j);if(y0>1 || y0< -1 || y1>1 || y1< -1)$fatal(1,"zero mean bias %0d/%0d",y0,y1);
    end
    tests=tests+1;$display("PASS alternating +/-4 zero mean");

    reset_case();kind=0;amp=0;noise=8;capture();energy=0;ground=0;
    for(j=0;j<64;j=j+1)begin read_bin(j);energy=energy+y0*y0;ground=ground+y0;end
    if($sqrt(energy/64.0)>2.0 || ground/64.0>0.75 || ground/64.0< -0.75)
      $fatal(1,"noise RMS/bias %f/%f",$sqrt(energy/64.0),ground/64.0);
    tests=tests+1;$display("PASS independent noise RMS=%f bias=%f",$sqrt(energy/64.0),ground/64.0);

    reset_case();kind=2;amp=4;capture();good=0;worst=0;
    for(j=0;j<64;j=j+1)begin
      read_bin(j);
      // Skip only the two bins that contain the ideal square's discontinuities.
      // All other plateau bins must retain +/-4 counts, no 15-count deadband.
      if(j!=0 && j!=32)begin
        v=(j<32)?4:-4;delta=y0-v;if(delta<0)delta=-delta;
        if(delta>worst)worst=delta;
        if(!valid || delta>1 || y1!= -y0)$fatal(1,"physical square bin %0d got %0d expected %0d",j,y0,v);
        good=good+1;
      end
    end
    tests=tests+1;$display("PASS physical 1 MHz weak square %0d plateau bins worst=%0d",good,worst);

    reset_case();kind=2;amp=20;signal_hz=5000000.0;fcw=32'd330382100;capture();
    if(filled>14 || ready>13 || ready<12)$fatal(1,"coherent grid falsely filled %0d/%0d",filled,ready);
    good=0;
    for(j=0;j<64;j=j+1)begin
      read_bin(j);if(valid)good=good+1;
      if(valid!=(n>=128))$fatal(1,"bin mask/count mismatch");
      snap_y[j]=y0;snap_v[j]=valid;snap_n[j]=n;
    end
    if(good!=ready)$fatal(1,"ready statistic mismatch");
    tests=tests+1;$display("PASS exact physical 5 MHz grid ready=%0d/64 (gaps preserved)",ready);
    // Clear LIVE state mid-transmission; the frozen frame must remain atomic.
    locked=0;ref_ok=0;commit=1;repeat(3)@(negedge clk);commit=0;
    for(j=0;j<64;j=j+1)begin
      read_bin(j);if(y0!==snap_y[j] || valid!==snap_v[j] || n!==snap_n[j])
        $fatal(1,"snapshot changed under reader");
    end
    if(dut.live_hits!=0)$fatal(1,"unlocked accumulator not cleared");
    tests=tests+1;$display("PASS immutable frame under lock loss / commit");

    reset_case();kind=2;amp=20;locked=0;ref_ok=0;capture();
    if(ready!=0 || filled!=0 || sq || sl || hits!=0)$fatal(1,"unlocked snapshot looked valid");
    tests=tests+1;$display("PASS unlocked publishes only invalid gaps");

    // Negative control from the old test: actual 5 MHz, NCO +5 kHz. The
    // independently calculated residual is -5 kHz, NOT a way to fill bins.
    reset_case();kind=2;amp=20;signal_hz=5000000.0;fcw=32'd330712482;
    residual= -32'sd330382;capture();
    if(ready!=0 || filled!=0 || sq)$fatal(1,"NCO error accepted as phase diversity");
    tests=tests+1;$display("PASS +5 kHz NCO error rejected, never a full-coverage success");

    // Positive control: the PHYSICAL signal and its tracked NCO are both
    // 5.005 MHz. The new positions are real observations, unlike the negative
    // control above. Assert true waveform plateaus, not a copied EMA equation.
    reset_case();kind=2;amp=4;signal_hz=5005000.0;fcw=32'd330712482;
    capture();
    if(filled!=64 || ready!=64)$fatal(1,"real phase diversity incomplete %0d/%0d",filled,ready);
    for(j=1;j<64;j=j+1)if(j!=32)begin
      read_bin(j);v=(j<32)?4:-4;
      if(!valid || y0!=v || y1!= -v)$fatal(1,"real drift waveform distorted bin %0d=%0d",j,y0);
    end
    tests=tests+1;$display("PASS physical 5.005 MHz + tracked NCO: full bins AND correct weak waveform");

    reset_case();kind=0;amp=4;
    for(k=0;k<50000;k=k+1)begin
      step_sample();
      if(dut.phase_spent>=dut.PHASE_BUDGET)$fatal(1,"phase budget exceeded");
    end
    if(dut.live_epoch<2)$fatal(1,"phase history never expired");
    old_epoch=dut.live_epoch;
    @(negedge clk);sv=0;repeat(3)@(negedge clk);
    if(dut.live_hits!=0)$fatal(1,"missing sample did not clear epoch");
    tests=tests+1;$display("PASS bounded phase history / missing-sample invalidation epoch=%0d",old_epoch);
    $display("TB_PASS tb_et_integrity tests=%0d",tests);$finish;
  end
  initial begin #30000000;$fatal(1,"timeout");end
endmodule
`default_nettype wire
