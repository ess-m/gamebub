package platform.handheld

import chisel3._
import chisel3.util._
import _root_.circt.stage.ChiselStage
import lib.audio.{AudioResampler, AudioResamplerParams}
import lib.mem.sdram.{BurstSdramController, Signals => SdramSignals}
import lib.mem.{HandshakeMemoryCdc, MemoryArbiter, MemoryInterface, MemoryMap, PipelineInterfaceBridge, PipelineMemoryArbiter, PipelineMemoryBurstCdc, PipelineMemoryInterface, RegisterMap}
import lib.video.{Color, ColorARGB, ColorCorrection, ColorGrayscale}
import xilinx.{XpmCdcHandshake, XpmCdcSingle, XpmCdcSyncRst}
import xilinx.MMCM
import net.gamebub.framework.interface._

object HandheldTop extends App {
  // Parse arguments.
  if (args.length < 2) {
    throw new IllegalArgumentException("missing arg 0: inner class, arg 1: revision")
  }
  val argInnerClassName :: argRevision :: argRest = args.toList

  // Generate verilog.
  val moduleFactory = () =>
    Class
      .forName(argInnerClassName)
      .getDeclaredConstructor()
      .newInstance()
      .asInstanceOf[Module with HandheldModule]

  ChiselStage.emitSystemVerilogFile(
    new HandheldTop(moduleFactory(), getRevision(argRevision)),
    argRest.toArray,
    firtoolOpts = Array(
      "--preserve-aggregate=1d-vec",
    )
  )

  private def getRevision(name: String): Revision = {
    name match {
      case "1" | "2" => Revision(
        displayWidth = 480,
        displayHeight = 320,
        displayRotate = true,
        displayColorDepth = 6,
        dpiConfig = AdaptiveDpiDriver.Config(
          clockHz = 12_288_000,
          hActive = 320,
          vActive = 480,
          variableVsync = true,
          hSyncMin = 3,
          hBackPorchMin = 3,
          hFrontPorchMin = 3,
          vSyncMin = 1,
          vBackPorchMin = 2,
          vFrontPorchMin = 2,
          // vsync + vbp + vfp < 32
          vFrontPorchMax = 32 - 1 - 2 - 1,
        ),
        overlayWidth = 240,
        overlayHeight = 160,
        dpiClockDivider = 76,
        audioMclkFactor = 256,
      )
      case "3" => Revision(
        displayWidth = 800,
        displayHeight = 480,
        displayColorDepth = 6,
        dpiConfig = AdaptiveDpiDriver.Config(
          clockHz = 26_100_000,
          hActive = 800,
          vActive = 480,
          variableVsync = false,
          hSyncMin = 2,
          hBackPorchMin = 4,
          hFrontPorchMin = 4,
          vSyncMin = 2,
          vBackPorchMin = 4,
          vFrontPorchMin = 4,
        ),
        overlayWidth = 360,
        overlayHeight = 240,
        dpiClockDivider = 36,
        audioMclkFactor = 544,
      )
      case "4" => Revision(
        displayWidth = 800,
        displayHeight = 480,
        displayRotate = true,
        displayOffsetX = -28,
        displayColorDepth = 8,
        dpiConfig = AdaptiveDpiDriver.Config(
          clockHz = 29_362_000,
          hActive = 480,
          vActive = 800,
          variableVsync = true,
          hSyncMin = 4,
          hBackPorchMin = 10,
          hFrontPorchMin = 47,
          vSyncMin = 4,
          vBackPorchMin = 20,
          vFrontPorchMin = 10,
          vFrontPorchMax = 255,
        ),
        overlayWidth = 360,
        overlayHeight = 240,
        dpiClockDivider = 32,
        audioMclkFactor = 608,
      )
      case _ => throw new IllegalArgumentException("invalid revision " + name)
    }
  }
}

/** IO bundle used for a handheld submodule. */
abstract class HandheldIo extends Bundle {
  val clocks: ClocksFixedV0
  val video: VideoV0
  val audio: AudioV0
  val host: HostV0
  val pmod: PmodV0
  val input: InputV0
  val cartridge: CartridgePortV0
  val link: LinkPortV0
  val sram: SramV0
  val sdram: SdramV0
}

trait HandheldModule {
  def io: HandheldIo
}

class HandheldInterrupts extends Bundle {
  val spiResponseFifoUnderflow = Bool()
  val spiRequestFifoOverflow = Bool()
  val buttonEdge = Bool()
  val moduleVblank = Bool()
}

object HandheldVibrate extends ChiselEnum {
  val Off, On, Brake = Value
}

/**
 * Top-level Chisel module for the handheld.
 *
 * The outer clock is passed down to the inner module,
 * e.g. 8.3886 MHz for Gameboy.
 */
class HandheldTop[T <: Module with HandheldModule](genT: => T, revision: Revision) extends Module {
  val module = Module(genT)
  val sdramConfig = BurstSdramController.Config(
    clockFrequency = module.io.clocks.clockSdramHz,
    accessLength = 2,
    timeRsc = (2 * 1_000_000_000) / module.io.clocks.clockSdramHz, /* 2 clocks */
    timeWr = (2 * 1_000_000_000) / module.io.clocks.clockSdramHz, /* 2 clocks */
    enableBurst = module.io.sdram.sdramBurst,
  )
  val io = IO(new Bundle {
    /** Clocking **/
    val clockIn50Mhz = Input(Clock())
    val clockOutSys = Output(Clock())
    val clockOutDpi = Output(Clock())
    val clockOutLocked = Output(Bool())

    /** Audio/video clock: DPI when HDMI disabled, 27.027 MHz when HDMI enabled */
    val clock_av = Input(Clock())

    /** MCU interrupt: true to pull it low (active) */
    val mcuIrq = Output(Bool())
    val mcuSpiChipSelect = Input(Bool())
    val mcuSpiClock = Input(Bool())
    val mcuSpiDataIn = Input(UInt(4.W))
    val mcuSpiDataOut = Output(UInt(4.W))
    val mcuSpiDataDir = Output(UInt(4.W))

    val lcd = Output(new DpiSignals)
    val lcdDataR = Output(UInt(revision.displayColorDepth.W))
    val lcdDataG = Output(UInt(revision.displayColorDepth.W))
    val lcdDataB = Output(UInt(revision.displayColorDepth.W))
    val dac = Output(new I2sSignals)

    /** HDMI */
    val hdmiEnable = Output(Bool())
    val hdmiClockPowerDown = Output(Bool())
    val hdmiAudioClock = Output(Clock())
    val hdmiAudio = Output(Vec(2, UInt(16.W)))
    val hdmiRgb = Output(UInt(24.W))
    val hdmiCx = Input(UInt(10.W))
    val hdmiCy = Input(UInt(10.W))

    /** Raw button input, not registered or inverted. */
    val buttons = Input(new InputV0.Buttons)

    // Cartridge I/O
    val cartridge3V3Enable = Output(Bool())
    val cartridge5V0Enable = Output(Bool())

    val cartridge = new CartridgePortV0

    val vibrate = Output(Bool())
    val pmod = new PmodV0
    val link = new LinkPortV0

    // SRAM
    val sram = new AsyncSramController.Signals(addressWidth = 18, dataWidth = 16)

    // SDRAM
    val sdramClock = Output(Clock())
    val sdram = new SdramSignals(addressWidth = 13, dataWidth = 16, bankWidth = 2)
  })

  //////////////////////////////////
  // Main MMCM
  //////////////////////////////////
  val mmcm = Module(new MMCM(
    clockInHz = 50_000_000,
    divide = 3,
    multiply = 56.375,
    clockOutConfig = Seq(
      MMCM.ClockOut(module.io.clocks.sysDivider),
      MMCM.ClockOut(module.io.clocks.sdramDivider),
      MMCM.ClockOut(revision.dpiClockDivider),
      MMCM.ClockOut(5), // SPI, ~188 MHz
    )
  ))
  mmcm.io.clockIn := io.clockIn50Mhz
  mmcm.io.powerDown := false.B
  io.clockOutLocked := mmcm.io.locked
  io.clockOutSys := mmcm.io.clockOuts(0)
  val sdramClock = mmcm.io.clockOuts(1)
  io.clockOutDpi := mmcm.io.clockOuts(2)
  val clockSpi = mmcm.io.clockOuts(3)
  io.sdramClock := sdramClock

  //////////////////////////////////
  // MCU Communication
  //////////////////////////////////
  // D0: PICO, D1: POCI
  // TODO: clock gate when nCS is high
  val clockSpiLocked = Wire(Bool())
  val spi = Module(new SpiReceiverFifo())
  spi.io.clockSpi := clockSpi
  spi.io.clockSpiLocked := clockSpiLocked
  io.mcuSpiDataDir := Mux(io.mcuSpiChipSelect, 0.U, spi.io.signals.serialDir)
  io.mcuSpiDataOut := spi.io.signals.serialOut
  spi.io.signals.serialClock := io.mcuSpiClock
  spi.io.signals.serialIn := io.mcuSpiDataIn
  spi.io.signals.chipSelect := io.mcuSpiChipSelect
  withClock (clockSpi) {
    clockSpiLocked := RegNext(!spi.io.clockSpiPowerDown)
  }

  val controlRegister = RegInit(0.U.asTypeOf(new Bundle() {
    /** True to enable vibration (if the module uses it) */
    val vibrate = Bool()
    /** Whether the module is currently in vblank. (TODO make read-only) */
    val moduleVblank = Bool()
    /** Active-low reset for the inner module. */
    val moduleReset = Bool()
    /** Active-high enable for the inner module. */
    val moduleEnable = Bool()
  }))
  val displayRegister = RegInit(0.U.asTypeOf(new Bundle() {
    val docked = Bool()
  }))
  /// Buttons that are forced down by MCU
  val buttonForceRegister = RegInit(0.U.asTypeOf(new InputV0.Buttons))
  val interruptEnable = RegInit(0.U.asTypeOf(new HandheldInterrupts))
  val interruptFlags = RegInit(0.U.asTypeOf(new HandheldInterrupts))
  val statusRegister = Cat(
    // 0: cartridge switch state
    RegNext(RegNext(io.cartridge.switch)),
  )
  val colorCorrectionRegister = RegInit(0.U.asTypeOf(new Bundle() {
    val enableColorCorrections = Bool()
  }))

  val overlayXControlRegister = RegInit(0.U.asTypeOf(new Bundle() {
    val start = UInt(8.W)
    val end = UInt(8.W)
    val scroll = UInt(8.W)
  }))
  val overlayYControlRegister = RegInit(0.U.asTypeOf(new Bundle() {
    val start = UInt(8.W)
    val end = UInt(8.W)
    val scroll = UInt(8.W)
  }))
  /// Synchronized physical button state (without MCU force override)
  val buttonState = Wire(new InputV0.Buttons)

  val registerMap = RegisterMap(
    addressWidth = 16,
    dataWidth = 32,
    entries = Seq(
      0x0 -> RegisterMap.Entry.rw(controlRegister),
      0x4 -> RegisterMap.Entry.rw(buttonForceRegister),
      0x8 -> RegisterMap.Entry.rw(displayRegister),
      0xC -> RegisterMap.Entry.rw(interruptEnable),
      0x10 -> RegisterMap.Entry(
        interruptFlags.getWidth,
        read = RegisterMap.ReadFn((_: Bool) => interruptFlags.asUInt),
        write = RegisterMap.WriteFn((write: Bool, data: UInt) =>
          when (write) {
            // Write set bits to ack interrupts.
            interruptFlags := (interruptFlags.asUInt & (~data).asUInt).asTypeOf(interruptFlags)
          }
        ),
      ),
      0x14 -> RegisterMap.Entry.r(statusRegister),
      0x18 -> RegisterMap.Entry.rw(colorCorrectionRegister),
      0x1C -> RegisterMap.Entry.r(buttonState),
      // Overlay control
      0x100 -> RegisterMap.Entry.rw(overlayXControlRegister),
      0x104 -> RegisterMap.Entry.rw(overlayYControlRegister),
      // Framebuffer dimensions
      0x200 -> RegisterMap.Entry.r(
        Cat(module.io.video.videoWidth.U(16.W), module.io.video.videoHeight.U(16.W))),
      // Stats
      0x300 -> RegisterMap.Entry.r(0.U),
      0x304 -> RegisterMap.Entry.r(0.U),
    )
  )

  val sramSpiInterface = Wire(new MemoryInterface(addressWidth = 19, dataWidth = 16))
  val sdramSpiInterface = Wire(new MemoryInterface(addressWidth = 25, dataWidth = 32))
  val moduleMcuInterface = Wire(new MemoryInterface(addressWidth = 30, dataWidth = 32))
  val overlayInterface = Wire(new MemoryInterface(addressWidth = 18, dataWidth = 16))
  val framebufferInterface = Wire(new MemoryInterface(addressWidth = 18, dataWidth = 16))
  val colorCorrectInterface = Wire(new MemoryInterface(addressWidth = 9, dataWidth = 16))
  // 16 bit prefix: 64 KiB
  // 12 bit prefix: 1 MiB
  // 8 bit prefix: 16 MiB
  // 4 bit prefix: 256 MiB
  spi.io.mem <> MemoryMap(
    addressWidth = 32,
    dataWidth = 32,
    entries = Seq(
      0x01.U(8.W) -> registerMap,
      0x02.U(8.W) -> colorCorrectInterface,
      0x03.U(8.W) -> overlayInterface,
      0x04.U(8.W) -> framebufferInterface,
      0x05.U(8.W) -> sramSpiInterface,
      0x8.U(4.W) -> sdramSpiInterface,
      0xE.U(4.W) -> moduleMcuInterface,
    ))

  controlRegister.moduleVblank := module.io.video.vblank
  when (spi.io.debugRequestOverflow) {
    interruptFlags.spiRequestFifoOverflow := true.B

  }
  when (spi.io.debugResponseUnderflow) {
    interruptFlags.spiResponseFifoUnderflow := true.B
  }

  //////////////////////////////////
  // Interrupts
  //////////////////////////////////
  io.mcuIrq := (interruptFlags.asUInt & interruptEnable.asUInt).orR
  when (module.io.video.vblank && !RegNext(module.io.video.vblank)) {
    interruptFlags.moduleVblank := true.B
  }

  //////////////////////////////////
  // Buttons
  //////////////////////////////////
  {
    // Invert and synchronize buttons
    val regButtons = RegNext(RegNext(~io.buttons.asUInt)).asTypeOf(new InputV0.Buttons)
    buttonState := regButtons

    when (regButtons.asUInt =/= RegNext(regButtons.asUInt)) {
      // Button edge, mark interrupt
      interruptFlags.buttonEdge := true.B
    }
  }

  //////////////////////////////////
  // Memory
  //////////////////////////////////
  val sram = Module(new AsyncSramController(addressWidth = 18, dataWidth = 16))
  val sramArbiter = Module(new MemoryArbiter(addressWidth = 18, dataWidth = 16, n = 2))
  io.sram <> sram.io.signals
  sram.io.mem <> sramArbiter.io.target
  sramArbiter.io.initiator(0) <> sramSpiInterface
  sramArbiter.io.initiator(0).address := sramSpiInterface.address >> 1  // SPI is byte addressed

  // SDRAM
  val sdramArbiter = Module(new PipelineMemoryArbiter(addressWidth = 25, dataWidth = 32, n = 2))

  {
    val bridge = Module(new PipelineInterfaceBridge(addressWidth = 25, dataWidth = 32))
    bridge.io.source <> sdramSpiInterface
    bridge.io.dest <> sdramArbiter.io.initiator(0)
  }

  val sdram = withClock(sdramClock) {
    Module(new BurstSdramController(sdramConfig))
  }
  io.sdram <> sdram.io.signals

  withClock (sdramClock) {
    val cdc = Module(new PipelineMemoryBurstCdc(
      addressWidth = 25,
      dataWidth = 32,
      addressBurstIncrement = 4,
      enablePrefetch = module.io.sdram.sdramBurst,
    ))
    cdc.io.slowClock := clock
    cdc.io.initiator <> sdramArbiter.io.target
    cdc.io.target <> sdram.io.mem
  }

  //////////////////////////////////
  // Video
  //////////////////////////////////
  val videoWidth = module.io.video.videoWidth
  val videoHeight = module.io.video.videoHeight

  io.hdmiEnable := displayRegister.docked

  // Double buffering
  val framebuffers = (0 until 2).map(_ =>
    SRAM(
      videoWidth * videoHeight, UInt(module.io.video.data.getWidth.W),
      readPortClocks = Seq(io.clock_av), writePortClocks = Seq(), readwritePortClocks = Seq(clock)
    )
  )
  /// Last completed frame
  val regLastFrameComplete = RegInit(0.U(1.W))

  val overlayWidth = revision.overlayWidth
  val overlayHeight = revision.overlayHeight
  val overlayFramebuffer = SRAM(
    overlayWidth * overlayHeight, UInt(module.io.host.overlayColorDepth2.getWidth.W),
    readPortClocks = Seq(io.clock_av), writePortClocks = Seq(clock), readwritePortClocks = Seq(),
  )

  // Keep HDMI MMCM powered for a few more cycles after switching away
  // from it to ensure the clock mux functions correctly.
  val hdmiClockPowerTimer = RegInit(0.U(3.W))
  when (displayRegister.docked) {
    hdmiClockPowerTimer := 7.U
  } .elsewhen (hdmiClockPowerTimer > 0.U) {
    hdmiClockPowerTimer := hdmiClockPowerTimer - 1.U
  }
  io.hdmiClockPowerDown := hdmiClockPowerTimer === 0.U

  val reset_av = withClock(io.clock_av) { XpmCdcSyncRst(reset) }
  withClockAndReset (clock = io.clock_av, reset = reset_av) {
    val videoX = Wire(UInt(10.W))
    val videoY = Wire(UInt(10.W))
    val framebufferReadAddress = Wire(UInt(log2Ceil(videoWidth * videoHeight).W))
    val overlayReadAddress = Wire(UInt(log2Ceil(overlayWidth * overlayHeight).W))

    // Raw audio CDC for the HDMI path: in docked mode clock_av is the external 27.027 MHz
    // HDMI clock, which is not derived from the MMCM VCO, so the exact-ratio resampler
    // below does not apply.
    val audioData = XpmCdcHandshake.continuous(clock, Cat(module.io.audio.left.asUInt, module.io.audio.right.asUInt))
    val audioDataLeft = audioData(31, 16)
    val audioDataRight = audioData(15, 0)

    // Buffering the read allows this to be a block ram instead of distributed ram
    // and an additional output buffer allows Vivado to improve timing.
    //
    // Read from the correct framebuffer.
    val framebufferIndex = Wire(UInt(1.W))
    val lastFrameComplete = XpmCdcSingle(clock, regLastFrameComplete.asBool).asUInt
    for (i <- 0 until 2) {
      framebuffers(i).readPorts(0).enable := framebufferIndex === i.U
      framebuffers(i).readPorts(0).address := framebufferReadAddress
    }
    val framebufferRead = MuxLookup(framebufferIndex, 0.U)(
      (0 until 2).map(i => i.U -> RegNext(RegNext(framebuffers(i).readPorts(0).data)))
    ).asTypeOf(module.io.video.data)

    // Color corrections
    val colorCorrector = Module(new ColorCorrection(inputDepth = 5, outputDepth = 6))
    colorCorrector.io.enable := XpmCdcSingle(clock, colorCorrectionRegister.enableColorCorrections)
    colorCorrector.io.in := framebufferRead
    val framebufferColor = colorCorrector.io.out

    {
      val cdc = Module(new HandshakeMemoryCdc(addressWidth = 9, dataWidth = 16))
      cdc.io.sourceClock := clock
      cdc.io.sourceReset := reset
      cdc.io.initiator <> colorCorrectInterface
      val mem = cdc.io.target
      mem.done := true.B
      mem.dataRead := DontCare
      val matrix = RegInit(VecInit(Seq(1, 0, 0, 0, 1, 0, 0, 0, 1).map(x => (x << 10).S(12.W))))
      val inputTable = RegInit(VecInit((0 until 32).map(i => {
        val normal = i.toDouble / 31.0
        (normal * 1024).floor.min(1023).toInt.S(11.W)
      })))
      val outputTable = RegInit(VecInit((0 until 64).map(i => {
        val normal = i.toDouble / 63.0
        (normal * 64).floor.min(63).toInt.U(6.W)
      })))
      when (mem.enable && mem.write) {
        when (mem.address(8, 7) === 0.U) {
          matrix(mem.address(4, 1)) := mem.dataWrite.asSInt
        }
        when (mem.address(8, 7) === 1.U) {
          inputTable(mem.address(5, 1)) := mem.dataWrite.asSInt
        }
        when (mem.address(8, 7) === 2.U) {
          outputTable(mem.address(6, 1)) := mem.dataWrite
        }
      }
      colorCorrector.io.matrixR := VecInit(matrix(0), matrix(1), matrix(2))
      colorCorrector.io.matrixG := VecInit(matrix(3), matrix(4), matrix(5))
      colorCorrector.io.matrixB := VecInit(matrix(6), matrix(7), matrix(8))
      colorCorrector.io.inputTable := inputTable
      colorCorrector.io.outputTable := outputTable
    }

    // Similar for overlay framebuffer.
    val overlayXControl = XpmCdcHandshake.continuous(clock, overlayXControlRegister)
    val overlayYControl = XpmCdcHandshake.continuous(clock, overlayYControlRegister)

    overlayFramebuffer.readPorts(0).enable := true.B
    overlayFramebuffer.readPorts(0).address := overlayReadAddress
    val overlayRead = RegNext(RegNext(overlayFramebuffer.readPorts(0).data))
      .asTypeOf(module.io.host.overlayColorDepth2)
      .convertTo(ColorARGB(1, 8, 8, 8))

    val framebufferInBounds = Wire(Bool())
    val overlayInBounds = Wire(Bool())
    val videoOutput = ColorARGB(0, 8, 8, 8).makeBlack()
    when (framebufferInBounds) {
      videoOutput := framebufferColor.convertTo(videoOutput)
    }
    when (overlayRead.a.asBool && overlayInBounds) {
      videoOutput := overlayRead.convertTo(videoOutput)
    }

    // DPI video signal output
    val dpiDriver = Module(new AdaptiveDpiDriver(
      config = revision.dpiConfig,
      sourceFramePeriod = module.io.video.framePeriod,
    ))
    dpiDriver.io.lastRenderedFrame := lastFrameComplete
    io.lcd := dpiDriver.io.signals
    val lcdData = videoOutput.convertTo(
      ColorARGB(0,
        revision.displayColorDepth,
        revision.displayColorDepth,
        revision.displayColorDepth,
      ))
    io.lcdDataR := lcdData.r
    io.lcdDataG := lcdData.g
    io.lcdDataB := lcdData.b

    val audioTransmitter =
      Module(new AudioDspTransmitter(
        bitWidth = 16,
        mclkFactor = revision.audioMclkFactor,
        channels = 2,
      ))
    io.dac := audioTransmitter.io.signals

    // Anti-aliased resampling from the core's native rate to the DAC sample rate.
    // The CIC front end runs in the core clock domain; the rest runs in this AV domain,
    // paced by the transmitter's sampleEnable.
    val audioResampler = Module(new AudioResampler(AudioResamplerParams(
      sysClockDivider = module.io.clocks.sysDivider,
      avClockDivider = revision.dpiClockDivider,
      audioMclkFactor = revision.audioMclkFactor,
    )))
    audioResampler.io.coreClock := clock
    audioResampler.io.coreReset := reset.asBool
    audioResampler.io.coreLeft := module.io.audio.left
    audioResampler.io.coreRight := module.io.audio.right
    audioResampler.io.sampleEnable := audioTransmitter.io.sampleEnable
    audioTransmitter.io.dataLeft := audioResampler.io.left.asUInt
    audioTransmitter.io.dataRight := audioResampler.io.right.asUInt
    // Flush buffered audio whenever the core is soft-reset (module swap) or the display
    // switches to HDMI, so no stale samples replay when the chain restarts.
    val moduleInReset = XpmCdcSingle(clock, !controlRegister.moduleReset)
    audioResampler.io.flush := moduleInReset

    /**
     * HDMI audio and video signal output
     * Video ID Code 2: 720x480 @ 60Hz
     */
    val hdmiFrameWidth = 858
    val hdmiFrameHeight = 525
    io.hdmiAudio := VecInit(audioDataLeft, audioDataRight)
    io.hdmiAudioClock := DontCare
    // Pad to 24-bit RGB.
    io.hdmiRgb := videoOutput.convertTo(ColorARGB(0, 8, 8, 8)).asUInt
    val regHdmiFrame = RegInit(0.U(1.W))

    val hdmiEnable = XpmCdcSingle(clock, displayRegister.docked)
    when (hdmiEnable) {
      dpiDriver.reset := true.B
      audioTransmitter.reset := true.B
      audioResampler.io.flush := true.B
      val screenWidth = 720
      val screenHeight = 480

      // Correct HDMI video X and Y
      videoX := io.hdmiCx
      videoY := io.hdmiCy
      when (io.hdmiCx >= screenWidth.U) {
        // Make it so that adding wraps around to 0.
        // (frameWidth - 1) should be (2**width - 1)
        videoX := io.hdmiCx + ((1 << io.hdmiCx.getWidth) - hdmiFrameWidth).U
        videoY := io.hdmiCy + 1.U
        when (io.hdmiCy === (hdmiFrameHeight - 1).U) {
          videoY := 0.U
        }
      }
      val hdmiFramePulse = io.hdmiCy === (hdmiFrameHeight - 1).U
      framebufferIndex := regHdmiFrame
      when (hdmiFramePulse && !RegNext(hdmiFramePulse)) {
        regHdmiFrame := lastFrameComplete
      }

      // Scale and center framebuffer within output video.
      val videoScale = (screenWidth / videoWidth).min(screenHeight / videoHeight)
      val videoOffsetX = (screenWidth - (videoWidth * videoScale)) / 2
      val videoOffsetY = (screenHeight - (videoHeight * videoScale)) / 2
      val framebufferReadDelay = 3 /* reading */ + 3 /* color corrections */
      framebufferReadAddress :=
        (((videoY - videoOffsetY.U) / videoScale.U) * videoWidth.U) +
          ((videoX - videoOffsetX.U + framebufferReadDelay.U) / videoScale.U)
      framebufferInBounds := videoX >= videoOffsetX.U &&
        videoX < (videoOffsetX + (videoWidth * videoScale)).U &&
        videoY >= videoOffsetY.U &&
        videoY < (videoOffsetY + (videoHeight * videoScale)).U

      // Scale overlay
      val overlayScale = (screenWidth / overlayWidth).min(screenHeight / overlayHeight)
      val overlayOffsetX = (screenWidth - (overlayWidth * overlayScale)) / 2
      val overlayOffsetY = (screenHeight - (overlayHeight * overlayScale)) / 2
      val overlayReadDelay = 3
      overlayReadAddress :=
        (((videoY - overlayOffsetY.U) / overlayScale.U)(8, 0) * overlayWidth.U) +
          ((videoX - overlayOffsetX.U + overlayReadDelay.U) / overlayScale.U)(8, 0)
      overlayInBounds :=
        videoX >= overlayOffsetX.U &&
          videoX < (overlayOffsetX + (overlayWidth * overlayScale)).U &&
          videoY >= overlayOffsetY.U &&
          videoY < (overlayOffsetY + (overlayHeight * overlayScale)).U

      // HDMI Audio
      val audioClock = RegInit(false.B)
      val audioCounter = Counter(27027000 / (48000 * 2))
      when (audioCounter.inc()) {
        audioClock := !audioClock
      }
      io.hdmiAudioClock := audioClock.asClock
    } .otherwise {
      val screenWidth = revision.displayWidth
      val screenHeight = revision.displayHeight

      val dpiX = if (revision.displayRotate) dpiDriver.io.pixelY else dpiDriver.io.pixelX
      val dpiY = if (revision.displayRotate) dpiDriver.io.pixelX else dpiDriver.io.pixelY
      videoX := dpiX
      videoY := dpiY
      framebufferIndex := dpiDriver.io.displayFrame

      // Scale and center framebuffer without output video.
      val videoScale = (screenWidth / videoWidth).min(screenHeight / videoHeight)
      val videoOffsetX = (screenWidth - (videoWidth * videoScale)) / 2 + revision.displayOffsetX
      val videoOffsetY = (screenHeight - (videoHeight * videoScale)) / 2
      val framebufferReadDelay = 3 /* reading */ + 3 /* color corrections */
      val framebufferReadDelayX = if (revision.displayRotate) 0 else framebufferReadDelay
      val framebufferReadDelayY = if (revision.displayRotate) framebufferReadDelay else 0
      framebufferReadAddress :=
        (((dpiY - videoOffsetY.U + framebufferReadDelayY.U) / videoScale.U) * videoWidth.U) +
          ((dpiX - videoOffsetX.U + framebufferReadDelayX.U) / videoScale.U)
      framebufferInBounds :=
        dpiX >= videoOffsetX.U &&
        dpiX < (videoOffsetX + (videoWidth * videoScale)).U &&
        dpiY >= videoOffsetY.U &&
        dpiY < (videoOffsetY + (videoHeight * videoScale)).U

      // Scale overlay
      val overlayScale = (screenWidth / overlayWidth).min(screenHeight / overlayHeight)
      val overlayOffsetX = (screenWidth - (overlayWidth * overlayScale)) / 2 + revision.displayOffsetX
      val overlayOffsetY = (screenHeight - (overlayHeight * overlayScale)) / 2
      val overlayReadDelay = 3
      val overlayReadDelayX = if (revision.displayRotate) 0 else overlayReadDelay
      val overlayReadDelayY = if (revision.displayRotate) overlayReadDelay else 0
      overlayReadAddress :=
        (((dpiY - overlayOffsetY.U + overlayReadDelayY.U) / overlayScale.U)(8, 0) * overlayWidth.U) +
          ((dpiX - overlayOffsetX.U + overlayReadDelayX.U) / overlayScale.U)(8, 0)
      overlayInBounds :=
        dpiX >= overlayOffsetX.U &&
        dpiX < (overlayOffsetX + (overlayWidth * overlayScale)).U &&
        dpiY >= overlayOffsetY.U &&
        dpiY < (overlayOffsetY + (overlayHeight * overlayScale)).U
      // TODO: re-add overlay X/Y positioning control if needed
    }
  }

//  io.pmod.dir := "b1111".U
//  module.io.pmod.in := 0.U

  // Overlay access.
  // TODO: consider switching to (or adding) a method of writing where
  // there's a "target x" and "target y" register, and you write to a single
  // memory location, which auto-increments the x. Then, have registers for
  // minX (where it wraps to) and maxX (when it wraps), which allows for easy
  // partial rectangular updates.
  overlayInterface.dataRead := DontCare
  overlayInterface.done := false.B
  overlayFramebuffer.writePorts(0).enable := overlayInterface.enable && overlayInterface.write
  overlayFramebuffer.writePorts(0).address := (overlayInterface.address >> 1).asUInt
  overlayFramebuffer.writePorts(0).data :=
    overlayInterface.dataWrite
      .asTypeOf(ColorARGB.argb1555())
      .convertTo(module.io.host.overlayColorDepth2)
      .asUInt
  overlayInterface.done := RegNext(overlayInterface.enable)

  // Framebuffer read via SPI.
  for (i <- 0 until 2) {
    framebuffers(i).readwritePorts(0).enable := false.B
    framebuffers(i).readwritePorts(0).address := DontCare
    framebuffers(i).readwritePorts(0).isWrite := DontCare
    framebuffers(i).readwritePorts(0).writeData := DontCare
  }
  val framebufferInterfaceRead = framebufferInterface.enable && !framebufferInterface.write
  when (framebufferInterfaceRead) {
    for (i <- 0 until 2) {
      when (regLastFrameComplete === i.U) {
        framebuffers(i).readwritePorts(0).enable := true.B
        framebuffers(i).readwritePorts(0).address := (framebufferInterface.address >> 1.U).asUInt
        framebuffers(i).readwritePorts(0).isWrite := false.B
      }
    }
  }
  framebufferInterface.dataRead := MuxLookup(regLastFrameComplete, 0.U)(
    (0 until 2).map(i => i.U ->
      RegNext(RegNext(framebuffers(i).readwritePorts(0).readData))
    ))
  framebufferInterface.done := RegNext(RegNext(framebufferInterface.enable))

  //////////////////////////////////
  // Submodule Connections
  //////////////////////////////////
  module.io.host.enable := controlRegister.moduleEnable
  module.io.host.reset := !controlRegister.moduleReset
  val vibrateEnabled = module.io.host.enable && controlRegister.vibrate && !displayRegister.docked
  io.vibrate := RegNext(module.io.input.vibrate === HandheldVibrate.On && vibrateEnabled)
  io.link <> module.io.link
  io.pmod <> module.io.pmod
  module.io.host.mem <> moduleMcuInterface
  module.io.input.buttons := (buttonState.asUInt | buttonForceRegister.asUInt).asTypeOf(new InputV0.Buttons)

  // Framebuffer writes
  {
    val framebufferX = RegInit(0.U(log2Ceil(module.io.video.videoWidth).W))
    val framebufferY = RegInit(0.U(log2Ceil(module.io.video.videoHeight).W))
    val framebufferWriteIndex = RegInit(0.U(1.W))

    when (module.io.video.dataEnable && !framebufferInterfaceRead) {
      // Module framebuffer write and SPI framebuffer read share the same read/write port,
      // so ensure that they're not activated at the same time (so they can be inferred correctly).
      val address = (framebufferY * module.io.video.videoWidth.U(10.W)) + framebufferX
      for (i <- 0 until 2) {
        framebuffers(i).readwritePorts(0).enable := (i.U === framebufferWriteIndex)
        framebuffers(i).readwritePorts(0).address := address
        framebuffers(i).readwritePorts(0).isWrite := true.B
        framebuffers(i).readwritePorts(0).writeData := module.io.video.data.asUInt
      }
    }

    val vblankEdge = module.io.video.vblank && !RegNext(module.io.video.vblank)
    val hblankEdge = module.io.video.hblank && !RegNext(module.io.video.hblank)
    when (vblankEdge) {
      regLastFrameComplete := framebufferWriteIndex
      framebufferWriteIndex := !framebufferWriteIndex
    }

    when (module.io.video.vblank) {
      // Frame ended
      framebufferX := 0.U
      framebufferY := 0.U
    } .elsewhen (module.io.video.hblank) {
      // Line ended
      when (hblankEdge) {
        framebufferX := 0.U
        framebufferY := framebufferY + 1.U
      }
    } .elsewhen (module.io.video.dataEnable) {
      framebufferX := framebufferX + 1.U
    }
  }

  // N.B. Audio synchronization happens above.

  // Cartridge
  io.cartridge <> module.io.cartridge
  // Rev1 and Rev2 only
  io.cartridge3V3Enable := RegNext(module.io.cartridge.enabled && !io.cartridge.switch)
  io.cartridge5V0Enable := RegNext(module.io.cartridge.enabled && io.cartridge.switch)

  // Memories
  sramArbiter.io.initiator(1) <> module.io.sram.mem
  sdramArbiter.io.initiator(1) <> module.io.sdram.mem
  module.io.clocks.clockSdram := sdramClock
}

case class Revision(
  displayWidth: Int,
  displayHeight: Int,
  displayRotate: Boolean = false,
  displayOffsetX: Int = 0,
  displayColorDepth: Int,
  dpiConfig: AdaptiveDpiDriver.Config,

  overlayWidth: Int,
  overlayHeight: Int,

  /// The AV clock divider
  dpiClockDivider: Int,
  /// The multipler to go from audio sample rate (48kHz) to MCLK
  audioMclkFactor: Int,
)