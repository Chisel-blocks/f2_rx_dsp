
// May  2018
// Last modification by Marko Kosunen, marko.kosunen@aalto.fi, 20.11.2018 11:47
//
// Input control parameters signals:
// The capacity of the output is users*datarate, so if we want to monitor all data 
// of all users from all  rx_paths independently, the rate has to be increased to
// antennas*users*datarate by serializing.
//
// input_mode:
//     0 : zero     : User data from the neighbouring modules is zeroed
//     1 : userssum : User data from neighbouring modules is summed to received user stream 
//                    of this module in rx_output_mode=4
//
// rx_ouput_mode: 
//     0 : Bypass        : User 0 from all rx paths is routed to output.
//     1 : Select user   : All received streams for user_index are selected 
//                         to output data stream
//     2 : Select antenna: All Users from  antenna_index are selected to ouput
//                         data stream   
//     3 : Select both   : User_index from antenna_index is selected to output 
//                         data stream position 0 
//     4 : Stream users  : In output data, Rx paths are in parallel, and users 
//                         are streamed out serial manner with higher data rate.  
//     5 : Stream rx     : In output data, Users are in parallel and Rx paths 
//                         are outputted in serial manner with higher data rate.
//     6 : Stream sum    : Output data stream is the sum of user data is. 
//                         Input_mode controls if the data from Neighbouring modules is included.
//
// user_index:
//     [0-users-1]       : Select the user of given index
//
// antenna_index:
//     [0-users-1]       : Select the antenna of given index
//
// adc_fifo_lut_mode     :
//
// inv_adc_clock_pol     :
//
// rx_user_delays        :
//
// rx_fine_delays        :
//
// rx_user_weights       :
//
// Clocking: 
//                       clock is the highest clock rate
//                       clock_symrate is the lowest clock rate, BB symbol rate
//               
/////////////////////////////////////////////////////////////////////////////
//
// TODO: Simplify clocking
//
//////////////////////////////////////////////////////////////////////////////////

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

//Constants
class usersigzeros(val n: Int, val users: Int=4) extends Bundle { 
    val userzero   = 0.U.asTypeOf(new usersigs(n=n,users=users))
    val udatazero  = 0.U.asTypeOf(userzero.data)
    val uindexzero = 0.U.asTypeOf(userzero.uindex)
    val iofifozero = 0.U.asTypeOf(new iofifosigs(n=n,users=users))
    val datazero   = 0.U.asTypeOf(iofifozero.data)
    val rxindexzero= 0.U.asTypeOf(iofifozero.rxindex)
}

class f2_rx_dsp_io(
        val inputn    : Int=9, 
        val n         : Int=16, 
        val resolution: Int=32, 
        val antennas  : Int=4, 
        val users     : Int=4,
        val neighbours: Int=4,
        val progdelay : Int=64,
        val finedelay : Int=32,
        val weightbits: Int=10
    ) extends Bundle {
    val iptr_A             = Input(Vec(antennas,DspComplex(SInt(inputn.W), SInt(inputn.W))))
    val decimator_clocks   =  new f2_decimator_clocks    
    val decimator_controls = Vec(antennas,new f2_decimator_controls(resolution=resolution,gainbits=10))    
    val adc_clocks         = Input(Vec(antennas,Clock()))
    val clock_symrate      = Input(Clock())
    val clock_symratex4    = Input(Clock()) //Should be x users if serial streaming is to be enabled
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
    val inv_adc_clk_pol    = Input(Vec(antennas,Bool()))
    val adc_lut_write_addr = Input(UInt(inputn.W))
    val adc_lut_write_vals = Input(Vec(antennas,DspComplex(SInt(inputn.W), SInt(inputn.W))))
    val adc_lut_write_en   = Input(Bool())
    val adc_lut_reset      = Input(Bool())
    val ofifo              = DecoupledIO(new iofifosigs(n=n,users=users))
    val iptr_fifo          = Vec(neighbours,Flipped(DecoupledIO(new iofifosigs(n=n,users=users))))
    val rx_user_delays     = Input(Vec(antennas, Vec(users,UInt(log2Ceil(progdelay).W))))
    val rx_fine_delays     = Input(Vec(antennas,UInt(log2Ceil(finedelay).W)))
    val rx_user_weights    = Input(Vec(antennas,Vec(users,DspComplex(SInt(weightbits.W),SInt(weightbits.W)))))
    val neighbour_delays   = Input(Vec(neighbours, Vec(users,UInt(log2Ceil(progdelay).W))))
}

class f2_rx_dsp (
        inputn     : Int=9,
        n          : Int=16, 
        resolution : Int=32, 
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
            resolution=resolution,
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
    val iofifozero = 0.U.asTypeOf(new iofifosigs(n=n,users=users))
    val datazero   = 0.U.asTypeOf(iofifozero.data)
    val rxindexzero= 0.U.asTypeOf(iofifozero.rxindex)

    //-The RX:s
    // Vec is required to do runtime adressing of an array i.e. Seq is not hardware structure
    val rx_path  = VecInit(Seq.fill(antennas){ 
            Module ( 
                new  f2_rx_path (
                    inputn=inputn,
                    n=n, 
                    resolution=resolution,
                    users=users, 
                    progdelay=progdelay,
                    finedelay=finedelay
                )
           ).io})
    
    (rx_path,io.decimator_controls).zipped.map(_.decimator_controls:=_)
    rx_path.map(_.decimator_clocks:=io.decimator_clocks) 
    (rx_path,io.iptr_A).zipped.map(_.iptr_A:=_)
    (rx_path,io.inv_adc_clk_pol).zipped.map(_.adc_ioctrl.inv_adc_clk_pol:=_)
    rx_path.map(_.adc_ioctrl.adc_fifo_lut_mode:=io.adc_fifo_lut_mode)
    rx_path.map(_.adc_ioctrl.adc_lut_write_addr:=io.adc_lut_write_addr)
    rx_path.map(_.adc_ioctrl.adc_lut_write_en:=io.adc_lut_write_en)
    rx_path.map(_.adc_ioctrl.adc_lut_reset:=io.adc_lut_reset)
    rx_path.map(_.adc_ioctrl.reset_adcfifo:=io.reset_adcfifo)
    (rx_path,io.adc_clocks).zipped.map(_.adc_clock:=_)
    (rx_path,io.adc_lut_write_vals).zipped.map(_.adc_ioctrl.adc_lut_write_val:=_)
    (rx_path,io.rx_user_delays).zipped.map(_.adc_ioctrl.user_delays:=_)
    (rx_path,io.rx_fine_delays).zipped.map(_.adc_ioctrl.fine_delays:=_)
    (rx_path,io.rx_user_weights).zipped.map(_.adc_ioctrl.user_weights:=_)

    //Input fifo from serdes
    val infifo = Seq.fill(neighbours){Module(new AsyncQueue(new iofifosigs(n=n, users=users),depth=fifodepth)).io}

    //Contains user indexes
    // usersigs is only for a single user therefore needs a Seq
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
    val w_Z = Wire( new iofifosigs(n=n,users=users))

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
    // r_iptr_fifo is a two dimensional Seq(neighbour,user)
    //form a seq of user data for every neighbour and sum them
    for (user <-0 to users-1){
        sumneighbourstream.data(user).udata:=r_iptr_fifo.map(
            neighbour => neighbour(user).optr_Z.data.udata
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
    val outfifo = Module(new AsyncQueue(new iofifosigs(n=n,users=users),depth=fifodepth)).io

    //Defaults
    outfifo.enq_reset:=io.reset_outfifo 
    outfifo.enq_clock:=io.clock_symrate
    outfifo.deq_reset:=io.reset_outfifo
    outfifo.deq.ready:=io.ofifo.ready
    outfifo.deq_clock:=io.clock_outfifo_deq
    io.ofifo.valid   := outfifo.deq.valid

    //Put something out if nothing else defined
    (w_Z.data,rx_path(0).Z).zipped.map(_.udata:=_)
     w_Z.data.map(_.uindex:=uindexzero)
     w_Z.rxindex:=rxindexzero

    //Mode operation definitions
    when( mode===bypass ) {
        (w_Z.data,rx_path(0).Z).zipped.map(_.udata:=_)
         w_Z.rxindex := rxindexzero
         outfifo.enq.valid :=  true.B   
         infifo.map(_.deq.ready :=  true.B)   
    }.elsewhen ( mode===select_users ) {
         w_Z:=RegNext(seluser)
         outfifo.enq.valid :=  true.B   
         infifo.map(_.deq.ready  :=  true.B)   
    }.elsewhen ( mode===select_antennas ) {
         w_Z:=RegNext(selrx)
         outfifo.enq.valid :=  true.B   
         infifo.map(_.deq.ready  :=  true.B)   
    }.elsewhen ( mode===select_both ) {
         w_Z:=RegNext(selrxuser)
         outfifo.enq.valid :=  true.B   
         infifo.map(_.deq.ready  :=  true.B)   
    }.elsewhen  (mode===stream_users ) {
         // Simplified, 16 users will blow the IO rate
         // Just another variant for debugging
         (w_Z.data,rx_path(1).Z).zipped.map(_.udata:=_)
         w_Z.rxindex := 1.U 
         outfifo.enq.valid :=  true.B   
         infifo.map(_.deq.ready :=  true.B)   
    }.elsewhen ( mode===stream_rx ) {
         //Simplified, became obsolete 16 users will blow the IO rate
         // Just another variant for debugging
         (w_Z.data,rx_path(2).Z).zipped.map(_.udata:=_)
         w_Z.rxindex := 2.U
         outfifo.enq.valid :=  true.B   
         infifo.map(_.deq.ready :=  true.B)   
    }.elsewhen ( mode===stream_sum ) {
         w_Z:= RegNext(sumusersstream)    
         outfifo.enq.valid :=  true.B   
         infifo.map(_.deq.ready  :=  true.B)   
    }.otherwise {
          w_Z  := RegNext(sumusersstream)
         outfifo.enq.valid :=  true.B   
         infifo.map(_.deq.ready  :=  true.B)   
    }
    
    //Here we reformat the output
    when ( outfifo.enq.ready ){
        outfifo.enq.bits:=w_Z
    } .otherwise {
        outfifo.enq.bits:=iofifozero
    }
    io.ofifo.bits :=  outfifo.deq.bits

}

//This gives you verilog
object f2_rx_dsp extends App {
  chisel3.Driver.execute(args, () => new f2_rx_dsp(inputn=9, resolution=32,n=16, antennas=4, users=16, fifodepth=128 ))
}

