library IEEE;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity LatheTop is
 generic (CLOCK_FREQUENCY   : natural := 50000000;  -- clock frequency of clk_i in Hz
          MEM_INT_IMEM_SIZE : natural := 16*1024;   -- size of processor-internal instruction memory in bytes
          MEM_INT_DMEM_SIZE : natural := 8*1024;    -- size of processor-internal data memory in bytes
          ledPins : positive := 8;
          dbgPins : positive := 8);
 port (
  sysClk   : in std_logic;
  rstn_i   : in std_ulogic;         -- global reset, low-active, async
  
  led      : out std_logic_vector(ledPins-1 downto 0) := (others => '0');
  dbg      : out std_logic_vector(dbgPins-1 downto 0) := (others => '0');
  anode    : out std_logic_vector(3 downto 0) := (others => '1');
  seg      : out std_logic_vector(6 downto 0) := (others => '1');

  dclk     : in std_logic;
  dout     : out std_logic := '0';
  din      : in std_logic;
  dsel     : in std_logic;

  aIn      : in std_logic;
  bIn      : in std_logic;
  syncIn   : in std_logic;

  zDro     : in std_logic_vector(1 downto 0);
  xDro     : in std_logic_vector(1 downto 0);
  zMpg     : in std_logic_vector(1 downto 0);

  xMpg     : in std_logic_vector(1 downto 0);

  pinIn    : in std_logic_vector(4 downto 0);

  -- aux      : out std_logic_vector(7 downto 0);
  aux      : out std_ulogic_vector(7 downto 0);

  pinOut   : out std_logic_vector(11 downto 0) := (others => '0');
  extOut   : out std_logic_vector(2 downto 0) := (others => '0');
  
  bufOut   : out std_logic_vector(3 downto 0) := (others => '0');

  zDoneInt : out std_logic := '0';
  xDoneInt : out std_logic := '0';

  -- JTAG on-chip debugger interface --
  jtag_trst_i : in  std_ulogic; -- low-active TAP reset (optional)
  jtag_tck_i  : in  std_ulogic; -- serial clock
  jtag_tdi_i  : in  std_ulogic; -- serial data input
  jtag_tdo_o  : out std_ulogic; -- serial data output
  jtag_tms_i  : in  std_ulogic; -- mode select

  -- GPIO --
  -- gpio_o      : out std_ulogic_vector(7 downto 0); -- parallel output
  
  -- UART0 --
  dbg_txd_o : out std_ulogic; -- UART0 send data
  dbg_rxd_i : in  std_ulogic  -- UART0 receive data
  );
end LatheTop;

architecture Behavioral of LatheTop is

 -- component neorv32_test_on_chip_debugger is
 --  generic (
 --   -- adapt these for your setup --
 --   CLOCK_FREQUENCY   : natural;  -- clock frequency of clk_i in Hz
 --   MEM_INT_IMEM_SIZE : natural;   -- size of processor-internal instruction memory in bytes
 --   MEM_INT_DMEM_SIZE : natural
 --   );
 --  port (
 --   -- Global control --
 --   clk_i       : in  std_ulogic; -- global clock, rising edge
 --   rstn_i      : in  std_ulogic; -- global reset, low-active, async
 --   -- JTAG on-chip debugger interface --
 --   jtag_trst_i : in  std_ulogic; -- low-active TAP reset (optional)
 --   jtag_tck_i  : in  std_ulogic; -- serial clock
 --   jtag_tdi_i  : in  std_ulogic; -- serial data input
 --   jtag_tdo_o  : out std_ulogic; -- serial data output
 --   jtag_tms_i  : in  std_ulogic; -- mode select
 --   -- GPIO --
 --   gpio_o      : out std_ulogic_vector(7 downto 0); -- parallel output
 --   -- UART0 --
 --   uart0_txd_o : out std_ulogic; -- UART0 send data
 --   uart0_rxd_i : in  std_ulogic  -- UART0 receive data
 --   );
 -- end Component;

 component LatheNew is
  generic (ledPins : positive;
           dbgPins : positive);
  port (
   sysClk   : in std_logic;

   led      : out std_logic_vector(ledPins-1 downto 0);
   dbg      : out std_logic_vector(dbgPins-1 downto 0);
   anode    : out std_logic_vector(3 downto 0);
   seg      : out std_logic_vector(6 downto 0);

   dclk     : in std_logic;
   dout     : out std_logic;
   din      : in std_logic;
   dsel     : in std_logic;

   aIn      : in std_logic;
   bIn      : in std_logic;
   syncIn   : in std_logic;

   zDro     : in std_logic_vector(1 downto 0);
   xDro     : in std_logic_vector(1 downto 0);
   zMpg     : in std_logic_vector(1 downto 0);

   xMpg     : in std_logic_vector(1 downto 0);

   pinIn    : in std_logic_vector(4 downto 0);

   -- aux      : out std_logic_vector(7 downto 0);
   pinOut   : out std_logic_vector(11 downto 0);
   extOut   : out std_logic_vector(2 downto 0);

   bufOut   : out std_logic_vector(3 downto 0);

   zDoneInt : out std_logic;
   xDoneInt : out std_logic
   );
 end Component;

 signal con_gpio_o : std_ulogic_vector(63 downto 0);

begin

neorv32_top_inst: neorv32_top
  generic map (
   -- General --
   CLOCK_FREQUENCY              => CLOCK_FREQUENCY,   -- clock frequency of clk_i in Hz
   --INT_BOOTLOADER_EN            => true,              -- boot configuration: true = boot explicit bootloader; false = boot from int/ext (I)MEM
   -- On-Chip Debugger (OCD) --
   ON_CHIP_DEBUGGER_EN          => true,              -- implement on-chip debugger
   -- RISC-V CPU Extensions --
   CPU_EXTENSION_RISCV_C        => true,              -- implement compressed extension?
   CPU_EXTENSION_RISCV_M        => true,              -- implement mul/div extension?
   CPU_EXTENSION_RISCV_Zicntr   => true,              -- implement base counters?
   CPU_EXTENSION_RISCV_Zifencei => true,              -- implement instruction stream sync.? (required for the on-chip debugger)
   -- Tuning Options --
   --CPU_IPB_ENTRIES              => 2,                 -- entries in instruction prefetch buffer, has to be a power of 2, min 1
   -- Internal Instruction memory --
   MEM_INT_IMEM_EN              => true,              -- implement processor-internal instruction memory
   MEM_INT_IMEM_SIZE            => MEM_INT_IMEM_SIZE, -- size of processor-internal instruction memory in bytes
   -- Internal Data memory --
   MEM_INT_DMEM_EN              => true,              -- implement processor-internal data memory
   MEM_INT_DMEM_SIZE            => MEM_INT_DMEM_SIZE, -- size of processor-internal data memory in bytes
   -- Processor peripherals --
   IO_GPIO_NUM                  => 8,                 -- number of GPIO input/output pairs (0..64)
   IO_MTIME_EN                  => true,              -- implement machine system timer (MTIME)?
   IO_UART0_EN                  => true               -- implement primary universal asynchronous receiver/transmitter (UART0)?
   )
  port map (

   -- clk_i       => clk_i,
   clk_i       => sysClk,
   rstn_i      => rstn_i,

   jtag_trst_i => jtag_trst_i,
   jtag_tck_i  => jtag_tck_i,
   jtag_tdi_i  => jtag_tdi_i,
   jtag_tdo_o  => jtag_tdo_o,
   jtag_tms_i  => jtag_tms_i,

   gpio_o      => con_gpio_o,

   uart0_txd_o => dbg_txd_o,
   uart0_rxd_i => dbg_rxd_i
);

  -- GPIO output --
  aux <= con_gpio_o(7 downto 0); 

 lathe: LatheInterface
  generic map (ledPins => ledPins,
               dbgPins => dbgPins)
  port map (
   sysClk   => sysClk,

   led      => led,
   dbg      => dbg,
   anode    => anode,
   seg      => seg,

   dclk     => dclk,
   dout     => dout,
   din      => din,
   dsel     => dsel,

   aIn      => aIn,
   bIn      => bIn,
   syncIn   => syncIn,

   zDro     => zDro,
   xDro     => xDro,
   zMpg     => zMpg,

   xMpg     => xMpg,

   pinIn    => pinIn,

   -- aux      => aux,
   pinOut   => pinOut,
   extOut   => extOut,

   bufOut   => bufOut,

   zDoneInt => zDoneInt,
   xDoneInt => xDoneInt
   );

end Behavioral;
