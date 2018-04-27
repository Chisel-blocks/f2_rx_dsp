// See LICENSE for license details.
//
//Start with a static tb and try to genererate a gnerator for it
package f2_rx_dsp
import chisel3._
import chisel3.util._
import chisel3.experimental._
import dsptools._
import dsptools.numbers._
import freechips.rocketchip.util._
import f2_decimator._
import f2_rx_path._

class iofifosigs(val n: Int) extends Bundle {
        val data=Vec(4,DspComplex(SInt(n.W), SInt(n.W)))
        val index=UInt(2.W)
}

class f2_rx_dsp_io(
        val inputn    : Int=9, 
        val n         : Int=16, 
        val antennas  : Int=4, 
        val users     : Int=4,
        val neighbours: Int=4
    ) extends Bundle {
    val iptr_A             = Input(Vec(antennas,DspComplex(SInt(inputn.W), SInt(inputn.W))))
    val decimator_clocks   =  new f2_decimator_clocks    
    val decimator_controls = Vec(antennas,new f2_decimator_controls(gainbits=10))    
    val adc_clocks         = Input(Vec(antennas,Clock()))
    val clock_symrate      = Input(Clock())
    val clock_symratex4    = Input(Clock())
    val clock_outfifo_deq  = Input(Clock())
    val clock_infifo_enq   = Input(Vec(neighbours,Clock()))
    val user_index         = Input(UInt(log2Ceil(users).W)) //W should be log2 of users
    val antenna_index      = Input(UInt(log2Ceil(antennas).W)) //W should be log2 of users
    val reset_index_count  = Input(Bool())
    val reset_adcfifo      = Input(Bool())
    val reset_outfifo      = Input(Bool())
    val reset_infifo       = Input(Bool())
    val rx_output_mode     = Input(UInt(3.W))
    val input_mode         = Input(UInt(3.W))
    val adc_fifo_lut_mode = Input(UInt(3.W))
    val adc_lut_write_addr = Input(UInt(inputn.W))
    val adc_lut_write_vals = Input(Vec(antennas,DspComplex(SInt(inputn.W), SInt(inputn.W))))
    val adc_lut_write_en   = Input(Bool())
    val ofifo              =  DecoupledIO(new iofifosigs(n=n))
    val iptr_fifo          =  Vec(neighbours,Flipped(DecoupledIO(new iofifosigs(n=n))))
}

class f2_rx_dsp (
        inputn    : Int=9,
        n         : Int=16, 
        antennas  : Int=4, 
        users     : Int=4, 
        fifodepth : Int=128, 
        neighbours: Int=4
    ) extends Module {
    val io = IO( new f2_rx_dsp_io(inputn=inputn,users=users)
    )
    val iozerovec=VecInit(Seq.fill(4)(DspComplex.wire(0.S(n.W), 0.S(n.W))))
    val iofifozerovec = Wire(new iofifosigs(n=n))
    iofifozerovec.data:=iozerovec
    iofifozerovec.index:=0.U

    //-The RX:s
    // Vec is required to do runtime adressing of an array i.e. Seq is not hardware structure
    val rx_path  = VecInit(Seq.fill(antennas){ Module ( new  f2_rx_path (n=n, users=users)).io})
    
    (rx_path,io.decimator_controls).zipped.map(_.decimator_controls:=_)
    rx_path.map(_.decimator_clocks:=io.decimator_clocks) 
    (rx_path,io.iptr_A).zipped.map(_.iptr_A:=_)
    rx_path.map(_.adc_fifo_lut_mode:=io.adc_fifo_lut_mode)
    rx_path.map(_.adc_lut_write_addr:=io.adc_lut_write_addr)
    rx_path.map(_.adc_lut_write_en:=io.adc_lut_write_en)
    rx_path.map(_.reset_adcfifo:=io.reset_adcfifo)
    (rx_path,io.adc_clocks).zipped.map(_.adc_clock:=_)
    (rx_path,io.adc_lut_write_vals).zipped.map(_.adc_lut_write_val:=_)

    //Input fifo from serdes
    //Reformulate to RX compliant format.
    val infifo = Seq.fill(neighbours){Module(new AsyncQueue(new iofifosigs(n=n),depth=fifodepth)).io}

    //Contains index, dunno if needed
    val r_iptr_fifo=  withClock(io.clock_symrate)(RegInit(VecInit(Seq.fill(neighbours)(iofifozerovec))))
    
    val zero :: userssum :: Nil = Enum(2)
    val inputmode=RegInit(zero)
    
    infifo.map(_.deq_reset:=io.reset_infifo)
    infifo.map(_.enq_reset:=io.reset_infifo)
    (infifo,io.iptr_fifo).zipped.map(_.enq<>_)
    (infifo,io.clock_infifo_enq).zipped.map(_.enq_clock:=_)
    infifo.map(_.deq_clock :=io.clock_symrate)
    when (io.input_mode===0.U) {
        inputmode := zero
        infifo.map(_.deq.ready:=false.B)
    } .elsewhen (io.input_mode===1.U ) {
        inputmode := userssum
        infifo.map(_.deq.ready:=true.B)
    } .otherwise {
        inputmode:=zero
        infifo.map(_.deq.ready:=false.B)
    }
   //--Serdes Input fifo ends here

    for (i <- 0 to neighbours-1) {
        when ( (infifo(i).deq.valid) && (inputmode===userssum)) { 
            r_iptr_fifo(i):=infifo(i).deq.bits
        } .elsewhen ( inputmode===zero ) {
            r_iptr_fifo(i):=iofifozerovec
        } .otherwise {
            r_iptr_fifo(i):=iofifozerovec
        }
    }

    //This is a 4 element receiver array, later to be concatenated to a bitvector
    val w_Z=  Wire(Vec(4,DspComplex(SInt(n.W), SInt(n.W))))
    val w_index=  Wire(UInt(2.W))
   
    // First we generate all possible output signals, then we just select The one we want.
    //Generate the sum of users
    val sumusersstream = withClockAndReset(io.clock_symrate,io.reset_outfifo)(
        RegInit(VecInit(Seq.fill(users)(DspComplex.wire(0.S(n.W), 0.S(n.W)))))
    )
    val sumneighbourstream = withClockAndReset(io.clock_symrate,io.reset_outfifo)(
        RegInit(VecInit(Seq.fill(users)(DspComplex.wire(0.S(n.W), 0.S(n.W)))))
    )
    
    //Sum the inputs from neighbours
    for (user <-0 to users-1){
        sumneighbourstream(user):=r_iptr_fifo.map(
            r_iptr_fifo=> r_iptr_fifo.data(user)
        ).foldRight(DspComplex(0.S(n.W), 0.S(n.W)))(
                (usrleft,usrright)=> usrleft+usrright
            )
    }

    //Sum neighbours to this receiver
    for (user <-0 to users-1){ 
        sumusersstream(user):=rx_path.map( 
            rxpath=> rxpath.Z(user)
        ).foldRight(sumneighbourstream(user))(
                (usrleft,usrright)=> usrleft+usrright
          )
    }
  
  
    //All antennas, single user
    val seluser = withClockAndReset(io.clock_symrate,io.reset_outfifo)(
        RegInit(VecInit(Seq.fill(antennas)(DspComplex.wire(0.S(n.W), 0.S(n.W)))))
    )
    (seluser,rx_path).zipped.map(_:=_.Z(io.user_index)) 

    //All users, single antenna
    val selrx = withClockAndReset(io.clock_symrate,io.reset_outfifo)(
        RegInit(VecInit(Seq.fill(users)(DspComplex.wire(0.S(n.W), 0.S(n.W)))))
    )
    (selrx,rx_path(io.antenna_index).Z).zipped.map(_:=_) 

    //Single users, single antenna
    val selrxuser = withClockAndReset(io.clock_symrate,io.reset_outfifo)(
        RegInit(DspComplex.wire(0.S(n.W), 0.S(n.W))))
    selrxuser:=rx_path(io.antenna_index).Z(io.user_index)
  

    //State counter to select the user or branch to the output
    val index=withClockAndReset(io.clock_symratex4,io.reset_outfifo)(
        RegInit(0.U(2.W))
    )
    when ( ! io.reset_index_count ) {
        when (index === 3.U) {
            index:=0.U
        } .otherwise {
            index := index+1.U(1.W)
        }
    } .otherwise {
        index := 0.U
    }
  
    // Indexed user stream
    val indexeduserstream = withClockAndReset(io.clock_symratex4,io.reset_outfifo)(
        RegInit(VecInit(Seq.fill(4)(DspComplex.wire(0.S(n.W), 0.S(n.W)))))
    )
    (indexeduserstream,rx_path).zipped.map(_:=_.Z(index))

    // Indexed RX stream
    val indexedrxstream = withClockAndReset(io.clock_symratex4,io.reset_outfifo)(
        RegInit(VecInit(Seq.fill(4)(DspComplex.wire(0.S(n.W), 0.S(n.W)))))
    )
    (indexedrxstream,rx_path(index).Z).zipped.map(_:=_)


    //Selection part starts here
    //State definiotions for the selected mode. Just to map numbers to understandable labels
    val ( bypass :: select_users  :: select_antennas :: select_both 
        :: stream_users :: stream_rx :: stream_sum :: Nil ) = Enum(7) 
    //Select state
    val mode=RegInit(bypass)
    
    //Decoder for the modes
    when(io.rx_output_mode===0.U){
        mode := bypass
    } .elsewhen(io.rx_output_mode===1.U) {
        mode := select_users
    } .elsewhen(io.rx_output_mode===2.U) {
        mode:=select_antennas
    } .elsewhen(io.rx_output_mode===3.U) {
        mode:=select_both
    } .elsewhen(io.rx_output_mode===4.U) {
        mode:=stream_users
    } .elsewhen(io.rx_output_mode===5.U) {
        mode:=stream_rx
    } .elsewhen(io.rx_output_mode===6.U) {
        mode:=stream_sum
    }.otherwise {
        mode := bypass
    }

    // Fifo for ther output
    val proto=UInt((4*2*n+2).W)
    val outfifo = Module(new AsyncQueue(new iofifosigs(n=n),depth=fifodepth)).io

    //Defaults
    outfifo.enq_reset:=io.reset_outfifo 
    outfifo.enq_clock:=io.clock_symratex4
    outfifo.deq_reset:=io.reset_outfifo
    outfifo.deq.ready:=io.ofifo.ready
    outfifo.deq_clock:=io.clock_outfifo_deq
    w_index := withClock(io.clock_symrate)(RegNext(0.U))
    io.ofifo.valid   := outfifo.deq.valid

    //Put something out if nothig else defined
    (w_Z,rx_path).zipped.map(_:=_.Z(0))

    //Clock multiplexing does not work. Use valid to control output rate.
    val validcount  = withClockAndReset(io.clock_symratex4,io.reset_outfifo)(RegInit(0.U(2.W)))
    val validreg =  withClockAndReset(io.clock_symratex4,io.reset_outfifo)(RegInit(false.B))
    //control the valid signal for the interface
    when ( (mode===bypass) ||  (mode===select_users) ||  (mode===select_antennas) 
        || (mode===select_both) || (mode===stream_sum)  ) {
        // In these modes, the write rate is symrate
        when (validcount===3.U) {
            validcount:=0.U
            validreg := true.B
        } .otherwise {
            validcount:= validcount+1.U(1.W)
            validreg := false.B
        }
    } .elsewhen ( ( mode===stream_users) || (mode===stream_rx) ) {
        // In these modes, the write rate is 4xsymrate
        validreg :=true.B
    } .otherwise {
        //Unknown modes
        validcount := 0.U
        validreg := false.B
    }
    outfifo.enq.valid :=  validreg   


    //Mode operation definitions
    switch(mode) {
        is(bypass) {
            (w_Z,rx_path).zipped.map(_:=_.Z(0))
            w_index := withClockAndReset(io.clock_symrate,io.reset_outfifo)(RegNext(0.U))
        }
        is(select_users) {
            (w_Z,seluser).zipped.map(_:=_)
            w_index := withClockAndReset(io.clock_symrate,io.reset_outfifo)(RegNext(io.user_index))
        }
        is(select_antennas) {
           (w_Z,selrx).zipped.map(_:=_)
            w_index := withClockAndReset(io.clock_symrate,io.reset_outfifo)(RegNext(io.antenna_index))
        }
        is(select_both) {
            (w_Z,Seq(selrxuser)++Seq.fill(3)(DspComplex.wire(0.S,0.S))).zipped.map(_:=_)
            w_index := withClockAndReset(io.clock_symrate,io.reset_outfifo)(RegNext(0.U))
        }
        is(stream_users) {
            w_Z := indexeduserstream    
            w_index := withClockAndReset(io.clock_symratex4,io.reset_outfifo)(RegNext(index))

        }
        is(stream_rx) {
            w_Z := indexedrxstream    
            w_index := withClockAndReset(io.clock_symratex4,io.reset_outfifo)(RegNext(index))
        }
        is(stream_sum) {
            w_Z := sumusersstream    
            w_index := withClock(io.clock_symrate)(RegNext(0.U))
        }
    }
    
    //Here we reformat the output signals to a single bitvector
    when ( outfifo.enq.ready ){
        outfifo.enq.bits.data:=w_Z
        outfifo.enq.bits.index:=w_index
    } .otherwise {
        outfifo.enq.bits.data:=iozerovec
        outfifo.enq.bits.index:=0.U
    }
    io.ofifo.bits.data :=  outfifo.deq.bits.data
    io.ofifo.bits.index := outfifo.deq.bits.index

}

//This gives you verilog
object f2_rx_dsp extends App {
  chisel3.Driver.execute(args, () => new f2_rx_dsp(inputn=9, n=16, antennas=4, users=4, fifodepth=128 ))
}

