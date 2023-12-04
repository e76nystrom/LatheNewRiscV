-- #################################################################################################
-- # << NEORV32 - Test Setup using the RISC-V-compatible On-Chip Debugger >>                       #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
-- #                                                                                               #
-- # Redistribution and use in source and binary forms, with or without modification, are          #
-- # permitted provided that the following conditions are met:                                     #
-- #                                                                                               #
-- # 1. Redistributions of source code must retain the above copyright notice, this list of        #
-- #    conditions and the following disclaimer.                                                   #
-- #                                                                                               #
-- # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
-- #    conditions and the following disclaimer in the documentation and/or other materials        #
-- #    provided with the distribution.                                                            #
-- #                                                                                               #
-- # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
-- #    endorse or promote products derived from this software without specific prior written      #
-- #    permission.                                                                                #
-- #                                                                                               #
-- # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
-- # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
-- # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
-- # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
-- # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
-- # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
-- # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
-- # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
-- # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
-- # ********************************************************************************************* #
-- # The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32                           #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity LatheRiscVTopDbg is
 generic (
  -- adapt these for your setup --
  CLOCK_FREQUENCY   : natural := 50000000;  -- clock frequency of clk_i in Hz
  MEM_INT_IMEM_SIZE : natural := 16*1024;   -- size of processor-internal instruction memory in bytes
  MEM_INT_DMEM_SIZE : natural := 8*1024;    -- size of processor-internal data memory in bytes
  ledPins : positive := 8;
  dbgPins : positive := 8
  );
 port (
  -- Global control --
  sysClk      : in  std_ulogic; -- global clock, rising edge
  rstn_i      : in  std_ulogic; -- global reset, low-active, async
  -- JTAG on-chip debugger interface --
  jtag_trst_i : in  std_ulogic; -- low-active TAP reset (optional)
  jtag_tck_i  : in  std_ulogic; -- serial clock
  jtag_tdi_i  : in  std_ulogic; -- serial data input
  jtag_tdo_o  : out std_ulogic; -- serial data output
  jtag_tms_i  : in  std_ulogic; -- mode select
  -- GPIO --
  --gpio_o      : out std_ulogic_vector(7 downto 0); -- parallel output
  -- UART0 --
  dbg_txd_o : out std_ulogic; -- UART0 send data
  dbg_rxd_i : in  std_ulogic; -- UART0 receive data

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

  aux      : out std_ulogic_vector(7 downto 0);
  pinOut   : out std_logic_vector(11 downto 0) := (others => '0');
  extOut   : out std_logic_vector(2 downto 0)  := (others => '0');
  bufOut   : out std_logic_vector(3 downto 0)  := (others => '0');

  zDoneInt : out std_logic := '0';
  xDoneInt : out std_logic := '0'
  );
end LatheRiscVTopDbg;

architecture Behavorial of LatheRiscVTopDbg is

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

 -- The Core Of The Problem ----------------------------------------------------------------
 -- -------------------------------------------------------------------------------------------
 neorv32_top_inst: entity neorv32.neorv32_top
  generic map (
   -- General --
   CLOCK_FREQUENCY              => CLOCK_FREQUENCY,   -- clock frequency of clk_i in Hz
   INT_BOOTLOADER_EN            => true,              -- boot configuration: true = boot explicit bootloader; false = boot from int/ext (I)MEM
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
   -- Global control --
   clk_i       => sysClk,      -- global clock, rising edge
   rstn_i      => rstn_i,      -- global reset, low-active, async
   -- JTAG on-chip debugger interface (available if ON_CHIP_DEBUGGER_EN = true) --
   jtag_trst_i => jtag_trst_i, -- low-active TAP reset (optional)
   jtag_tck_i  => jtag_tck_i,  -- serial clock
   jtag_tdi_i  => jtag_tdi_i,  -- serial data input
   jtag_tdo_o  => jtag_tdo_o,  -- serial data output
   jtag_tms_i  => jtag_tms_i,  -- mode select
   -- GPIO (available if IO_GPIO_NUM > 0) --
   gpio_o      => con_gpio_o,  -- parallel output
   -- primary UART0 (available if IO_UART0_EN = true) --
   uart0_txd_o => dbg_txd_o,   -- UART0 send data
   uart0_rxd_i => dbg_rxd_i    -- UART0 receive data
   );

 -- GPIO output --
 -- gpio_o <= con_gpio_o(7 downto 0);
 aux <= con_gpio_o(7 downto 0);

 latheNew0: LatheNew
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

end Behavorial;
