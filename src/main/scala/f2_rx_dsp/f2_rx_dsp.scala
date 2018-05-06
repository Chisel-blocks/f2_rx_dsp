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
import prog_delay._
import edge_detector._


//Consider using "unit" instead of "User" 
class usersigs (val n: Int, val users: Int=4) extends Bundle {
    val udata=DspComplex(SInt(n.W), SInt(n.W))
    val uindex=UInt(log2Ceil(users).W)
}

class iofifosigs(val n: Int, val users: Int=4 ) extends Bundle {
        //4=Users
        val data=Vec(users,new usersigs(n=n,users=users))
        val rxindex=UInt(2.W)
        override def cloneType = (new iofifosigs(n,users)).asInstanceOf[this.type]
}

//constants
class usersigzeros(val n: Int, val users: Int=4) extends Bundle { 
    val userzero   = 0.U.asTypeOf(new usersigs(n=n,users=users))
    val udatazero  = 0.U.asTypeOf(userzero.data)
    val uindexzero = 0.U.asTypeOf(userzero.uindex)
    val iofifozero = 0.U.asTypeOf(new iofifosigs(n=n))
    val datazero   = 0.U.asTypeOf(iofifozero.data)
    val rxindexzero= 0.U.asTypeOf(iofifozero.rxindex)
}

class f2_rx_dsp_io(
        val inputn    : Int=9, 
        val n         : Int=16, 
        val antennas  : Int=4, 
        val users     : Int=4,
        val neighbours: Int=4,
        val progdelay : Int=64,
        val finedelay : Int=32,
        val weightbits: Int=10
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
    val adc_fifo_lut_mode  = Input(UInt(3.W))
    val adc_lut_write_addr = Input(UInt(inputn.W))
    val adc_lut_write_vals = Input(Vec(antennas,DspComplex(SInt(inputn.W), SInt(inputn.W))))
    val adc_lut_write_en   = Input(Bool())
    val ofifo              = DecoupledIO(new iofifosigs(n=n))
    val iptr_fifo          = Vec(neighbours,Flipped(DecoupledIO(new iofifosigs(n=n,users=users))))
    val rx_user_delays     = Input(Vec(antennas, Vec(users,UInt(log2Ceil(progdelay).W))))
    val rx_fine_delays     = Input(Vec(antennas,UInt(log2Ceil(finedelay).W)))
    val rx_user_weights    = Input(Vec(antennas,Vec(users,DspComplex(SInt(weightbits.W),SInt(weightbits.W)))))
    val neighbour_delays   = Input(Vec(neighbours, Vec(users,UInt(log2Ceil(progdelay).W))))
}

class f2_rx_dsp (
        inputn     : Int=9,
        n          : Int=16, 
        antennas   : Int=4, 
        users      : Int=4, 
        fifodepth  : Int=128, 
        neighbours : Int=4,
        progdelay  : Int=64,
        finedelay  : Int=32,
        weightbits : Int=10
    ) extends Module {
    val io = IO( 
        new f2_rx_dsp_io(
            inputn=inputn,
            n=n,
            antennas=antennas,
            users=users,
            neighbours=neighbours,
            progdelay=progdelay,
            finedelay=finedelay
        )
    )
    //Zeros
    //val z = new usersigzeros(n=n, users=users)
    val userzero   = 0.U.asTypeOf(new usersigs(n=n,users=users))
    val udatazero  = 0.U.asTypeOf(userzero.data)
    val uindexzero = 0.U.asTypeOf(userzero.uindex)
    val iofifozero = 0.U.asTypeOf(new iofifosigs(n=n))
    val datazero   = 0.U.asTypeOf(iofifozero.data)
    val rxindexzero= 0.U.asTypeOf(iofifozero.rxindex)
    //-The RX:s
    // Vec is required to do runtime adressing of an array i.e. Seq is not hardware structure
    val rx_path  = VecInit(Seq.fill(antennas){ 
            Module ( 
                new  f2_rx_path (
                    inputn=inputn,
                    n=n, 
                    users=users, 
                    progdelay=progdelay,
                    finedelay=finedelay
                )
           ).io})
    
    (rx_path,io.decimator_controls).zipped.map(_.decimator_controls:=_)
    rx_path.map(_.decimator_clocks:=io.decimator_clocks) 
    (rx_path,io.iptr_A).zipped.map(_.iptr_A:=_)
    rx_path.map(_.adc_ioctrl.adc_fifo_lut_mode:=io.adc_fifo_lut_mode)
    rx_path.map(_.adc_ioctrl.adc_lut_write_addr:=io.adc_lut_write_addr)
    rx_path.map(_.adc_ioctrl.adc_lut_write_en:=io.adc_lut_write_en)
    rx_path.map(_.adc_ioctrl.reset_adcfifo:=io.reset_adcfifo)
    (rx_path,io.adc_clocks).zipped.map(_.adc_clock:=_)
    (rx_path,io.adc_lut_write_vals).zipped.map(_.adc_ioctrl.adc_lut_write_val:=_)
    (rx_path,io.rx_user_delays).zipped.map(_.adc_ioctrl.user_delays:=_)
    (rx_path,io.rx_fine_delays).zipped.map(_.adc_ioctrl.fine_delays:=_)
    (rx_path,io.rx_user_weights).zipped.map(_.adc_ioctrl.user_weights:=_)

    //Input fifo from serdes
    val infifo = Seq.fill(neighbours){Module(new AsyncQueue(new iofifosigs(n=n),depth=fifodepth)).io}

    //Contains indexes
    val delayproto=new usersigs(n=n,users=users)
    val r_iptr_fifo= Seq.fill(neighbours){ 
        Seq.fill(users){
            withClock(io.clock_symrate)( Module( new prog_delay(delayproto, maxdelay=progdelay)).io)
        }        
    }
    
    //Assign selects
    //This is the way to zip-map 2 dimensional arrays
    (r_iptr_fifo,io.neighbour_delays).zipped.map{ case(x,y)=> (x,y).zipped.map(_.select:=_)}

    
    val zero :: userssum :: Nil = Enum(2)
    val inputmode=RegInit(zero)
    
    infifo.map(_.deq_reset:=io.reset_infifo)
    infifo.map(_.enq_reset:=io.reset_infifo)
    (infifo,io.iptr_fifo).zipped.map(_.enq<>_)
    (infifo,io.clock_infifo_enq).zipped.map(_.enq_clock:=_)
    infifo.map(_.deq_clock :=clock)  //Fastest possible clock. Rate controlled by ready

    when (io.input_mode===0.U) {
        inputmode := zero
    } .elsewhen (io.input_mode===1.U ) {
        inputmode := userssum
    } .otherwise {
        inputmode:=zero
    }

    for (i <- 0 to neighbours-1) {
        when ( (infifo(i).deq.valid) && (inputmode===userssum)) {
            (r_iptr_fifo(i),infifo(i).deq.bits.data).zipped.map(_.iptr_A:=_)
        } .elsewhen ( inputmode===zero ) {
            r_iptr_fifo(i).map(_.iptr_A:=userzero) 
        } .otherwise {
            r_iptr_fifo(i).map(_.iptr_A:=userzero)
        }
    }
 
    //This is a wire for various mode assignments 
    val w_Z = Wire( new iofifosigs(n=n))

    // First we generate all possible output signals, then we just select The one we want.
    
    //Generate the sum of users
    val sumusersstream = withClockAndReset(io.clock_symrate,io.reset_outfifo)(
        RegInit(iofifozero)

    )
    sumusersstream.rxindex:=rxindexzero
    sumusersstream.data.map(_.uindex:=uindexzero)

    val sumneighbourstream = withClockAndReset(io.clock_symrate,io.reset_outfifo)(
        RegInit(iofifozero)
    )
    sumneighbourstream.rxindex:=rxindexzero
    sumneighbourstream.data.map(_.uindex:=uindexzero)
    
    //Sum the inputs from neighbours
    for (user <-0 to users-1){
        sumneighbourstream.data(user).udata:=r_iptr_fifo.map(
            r_iptr_fifo => r_iptr_fifo(user).optr_Z.data.udata
        ).foldRight(DspComplex(0.S(n.W), 0.S(n.W)))(
                (usrleft,usrright)=> usrleft+usrright
            )
    }
 
    //Sum neighbours to this receiver
    for (user <-0 to users-1){ 
        sumusersstream.data(user).udata:=rx_path.map( 
            rxpath=> rxpath.Z(user)
        ).foldRight(sumneighbourstream.data(user).udata)(
                (usrleft,usrright)=> usrleft+usrright
          )
    }
  
  
    //All antennas, single user
    val seluser = withClockAndReset(io.clock_symrate,io.reset_outfifo)(
        RegInit(iofifozero) //Includes index
    )
    (seluser.data,rx_path).zipped.map(_.udata:=_.Z(io.user_index)) 
     seluser.rxindex:=io.user_index

    //All users, single antenna now uindex is actually index for antenna
    val selrx = withClockAndReset(io.clock_symrate,io.reset_outfifo)(
        RegInit(iofifozero)
    )
    (selrx.data,rx_path(io.antenna_index).Z).zipped.map(_.udata:=_) 
     selrx.rxindex:=io.antenna_index

    //Single users, single antenna
    val selrxuser = withClockAndReset(io.clock_symrate,io.reset_outfifo)(
        RegInit(iofifozero))
        selrxuser.data(0).udata:=rx_path(io.antenna_index).Z(io.user_index)
        selrxuser.data(0).uindex:=io.user_index
        selrxuser.rxindex:=io.antenna_index
        selrxuser.data.drop(1).map(_.data:=userzero)
  

    //State counter to select the user or branch to the output
    val rxindex=withClockAndReset(io.clock_symratex4,io.reset_outfifo)(
        RegInit(0.U(2.W))
    )
    when ( ! io.reset_index_count ) {
        when (rxindex === 3.U) {
            rxindex:=0.U
        } .otherwise {
            rxindex := rxindex+1.U(1.W)
        }
    } .otherwise {
        rxindex := 0.U
    }
  
    // Indexed user stream
    val indexeduserstream = withClockAndReset(io.clock_symratex4,io.reset_outfifo)(
        RegInit(iofifozero)
    )
    (indexeduserstream.data,rx_path).zipped.map(_.udata:=_.Z(rxindex))
    indexeduserstream.data.map(_.uindex:=rxindex)
    indexeduserstream.rxindex:=rxindex

    // Indexed RX stream
    val indexedrxstream = withClockAndReset(io.clock_symratex4,io.reset_outfifo)(
        RegInit(iofifozero)
    )
    (indexedrxstream.data,rx_path(rxindex).Z).zipped.map(_.udata:=_)
    indexedrxstream.data.map(_.uindex:=rxindex)
    indexedrxstream.rxindex:=rxindex


    //Selection part starts here
    //State definitions for the selected mode. Just to map numbers to understandable labels
    val ( bypass :: select_users  :: select_antennas :: select_both 
        :: stream_users :: stream_rx :: stream_sum :: Nil ) = Enum(7) 
    //Select state
    val mode=RegInit(bypass)
    
    //Decoder for the modes
    when (io.rx_output_mode===0.U) {
        mode := bypass
    }.elsewhen (io.rx_output_mode===1.U) {
        mode := select_users
    }.elsewhen (io.rx_output_mode===2.U) {
        mode:=select_antennas
    }.elsewhen (io.rx_output_mode===3.U) {
        mode:=select_both
    }.elsewhen (io.rx_output_mode===4.U) {
        mode:=stream_users
    }.elsewhen (io.rx_output_mode===5.U) {
        mode:=stream_rx
    }.elsewhen (io.rx_output_mode===6.U) {
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
    io.ofifo.valid   := outfifo.deq.valid

    //Put something out if nothing else defined
    (w_Z.data,rx_path).zipped.map(_.udata:=_.Z(0))
     w_Z.data.map(_.uindex:=uindexzero)
     w_Z.rxindex:=rxindexzero

    //Clock multiplexing does not work. Use valid to control output rate.
    //TODO: change this to edge detector using clocks
    //val validcount  = withClockAndReset(io.clock_symratex4,io.reset_outfifo)(RegInit(uindexzero))
    //val validreg=  withClockAndReset(io.clock_symratex4,io.reset_outfifo)(RegInit(false.B))
    val edges_symratex4      =  Module( new edge_detector()).io 
    val edges_symrate =  Module( new edge_detector()).io 
    edges_symratex4.A :=io.clock_symratex4.asUInt
    edges_symrate.A :=io.clock_symrate.asUInt

    //control the valid signal for the interface
    //when ( (mode===bypass) ||  (mode===select_users) ||  (mode===select_antennas) 
    //    || (mode===select_both) || (mode===stream_sum)  ) {
    //    // In these modes, the write rate is symrate
    //    when (validcount===3.U) {
    //        validcount:=0.U
    //        validreg := true.B
    //    } .otherwise {
    //        validcount:= validcount+1.U(1.W)
    //        validreg := false.B
    //    }
    //} .elsewhen ( ( mode===stream_users) || (mode===stream_rx) ) {
    //    // In these modes, the write rate is 4xsymrate
    //    validreg :=true.B
    //} .otherwise {
    //    //Unknown modes
    //    validcount := 0.U
    //    validreg := false.B
    //}
    //outfifo.enq.valid :=  validreg   


    //Mode operation definitions
    when( mode===bypass ) {
        (w_Z.data,rx_path).zipped.map(_.udata:=_.Z(0))
         w_Z.rxindex := rxindexzero
         outfifo.enq.valid :=  true.B   
         infifo.map(_.deq.ready :=  true.B)   
    }.elsewhen ( mode===select_users ) {
         w_Z:=RegNext(seluser)
         outfifo.enq.valid :=  edges_symrate.rising   
         infifo.map(_.deq.ready  :=  edges_symrate.rising)   
    }.elsewhen ( mode===select_antennas ) {
         w_Z:=RegNext(selrx)
         outfifo.enq.valid :=  edges_symrate.rising   
         infifo.map(_.deq.ready  :=  edges_symrate.rising)   
    }.elsewhen ( mode===select_both ) {
         w_Z:=RegNext(selrxuser)
         outfifo.enq.valid :=  edges_symrate.rising   
         infifo.map(_.deq.ready  :=  edges_symrate.rising)   
    }.elsewhen  (mode===stream_users ) {
         w_Z := RegNext(indexeduserstream)    
         outfifo.enq.valid :=  edges_symratex4.rising   
         infifo.map(_.deq.ready  :=  edges_symratex4.rising) 
    }.elsewhen ( mode===stream_rx ) {
         w_Z := RegNext(indexedrxstream)    
         outfifo.enq.valid :=  edges_symratex4.rising   
         infifo.map(_.deq.ready  :=  edges_symratex4.rising) 
    }.elsewhen ( mode===stream_sum ) {
         w_Z:= RegNext(sumusersstream)    
         outfifo.enq.valid :=  edges_symrate.rising   
         infifo.map(_.deq.ready  :=  edges_symrate.rising)   
    }.otherwise {
          w_Z  := RegNext(sumusersstream)
         outfifo.enq.valid :=  edges_symrate.rising   
         infifo.map(_.deq.ready  :=  edges_symrate.rising)   
    }
    
    //Here we reformat the output signals to a single bitvector
    when ( outfifo.enq.ready ){
        outfifo.enq.bits:=w_Z
    } .otherwise {
        outfifo.enq.bits:=iofifozero
    }
    io.ofifo.bits :=  outfifo.deq.bits

}

//This gives you verilog
object f2_rx_dsp extends App {
  chisel3.Driver.execute(args, () => new f2_rx_dsp(inputn=9, n=16, antennas=4, users=4, fifodepth=128 ))
}

