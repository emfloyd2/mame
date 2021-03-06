// license:BSD-3-Clause
// copyright-holders:Robbbert
/***************************************************************************

    BEEHIVE DM3270

    25/05/2009 Skeleton driver [Robbbert]

    This is a conventional computer terminal using a serial link.
    Could be a clone of the IBM3276-2.

    The character gen rom is not dumped. Using the one from 'c10'
    for the moment.

    Screen goes crazy during the memory test, just ignore it.
    System freezes if ^G pressed.

    25/04/2011 Added partial keyboard.
    26/06/2011 Added modifier keys.

****************************************************************************/

#include "emu.h"
#include "cpu/i8085/i8085.h"
#include "screen.h"


class beehive_state : public driver_device
{
public:
	beehive_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_p_videoram(*this, "videoram")
		, m_p_chargen(*this, "chargen")
		{ }

	DECLARE_READ8_MEMBER(beehive_60_r);
	DECLARE_WRITE8_MEMBER(beehive_62_w);
	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

	void beehive(machine_config &config);
	void beehive_io(address_map &map);
	void beehive_mem(address_map &map);
private:
	required_device<cpu_device> m_maincpu;
	required_shared_ptr<uint8_t> m_p_videoram;
	required_region_ptr<u8> m_p_chargen;
	uint8_t m_keyline;
	virtual void machine_reset() override;
};

READ8_MEMBER(beehive_state::beehive_60_r)
{
	if (BIT(m_keyline, 4))
	{
		char kbdrow[6];
		sprintf(kbdrow,"X%d", m_keyline&15);
		return ioport(kbdrow)->read();
	}
	else
		return 0xff;
}

WRITE8_MEMBER(beehive_state::beehive_62_w)
{
	m_keyline = data;
}

void beehive_state::beehive_mem(address_map &map)
{
	map.unmap_value_high();
	map(0x0000, 0x17ff).rom();
	map(0x8000, 0x8fff).ram().share("videoram");
}

void beehive_state::beehive_io(address_map &map)
{
	map.global_mask(0xff);
	map.unmap_value_high();
	map(0x11, 0x11).portr("DIPS");
	map(0x60, 0x60).r(this, FUNC(beehive_state::beehive_60_r));
	map(0x61, 0x61).portr("MODIFIERS");
	map(0x62, 0x62).w(this, FUNC(beehive_state::beehive_62_w));
}

/* Input ports */
static INPUT_PORTS_START( beehive )
	PORT_START("X0")
		PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Q")  PORT_CODE(KEYCODE_Q)    PORT_CHAR('q') PORT_CHAR('Q')
		PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("W")  PORT_CODE(KEYCODE_W)    PORT_CHAR('w') PORT_CHAR('W')
		PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("E")  PORT_CODE(KEYCODE_E)    PORT_CHAR('e') PORT_CHAR('E')
		PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("R")  PORT_CODE(KEYCODE_R)    PORT_CHAR('r') PORT_CHAR('R')
		PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("T")  PORT_CODE(KEYCODE_T)    PORT_CHAR('t') PORT_CHAR('T')
		PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Y")  PORT_CODE(KEYCODE_Y)    PORT_CHAR('y') PORT_CHAR('Y')
		PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("U")  PORT_CODE(KEYCODE_U)    PORT_CHAR('u') PORT_CHAR('U')
		PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("I")  PORT_CODE(KEYCODE_I)    PORT_CHAR('i') PORT_CHAR('I')

	PORT_START("X1")
		PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("O")  PORT_CODE(KEYCODE_O)    PORT_CHAR('o') PORT_CHAR('O')
		PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("P")  PORT_CODE(KEYCODE_P)    PORT_CHAR('p') PORT_CHAR('P')
		PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("A")  PORT_CODE(KEYCODE_A)    PORT_CHAR('a') PORT_CHAR('A')
		PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("S")  PORT_CODE(KEYCODE_S)    PORT_CHAR('s') PORT_CHAR('S')
		PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("D")  PORT_CODE(KEYCODE_D)    PORT_CHAR('d') PORT_CHAR('D')
		PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F")  PORT_CODE(KEYCODE_F)    PORT_CHAR('f') PORT_CHAR('F')
		PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("G")  PORT_CODE(KEYCODE_G)    PORT_CHAR('g') PORT_CHAR('G')
		PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("H")  PORT_CODE(KEYCODE_H)    PORT_CHAR('h') PORT_CHAR('H')

	PORT_START("X2")
		PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J")  PORT_CODE(KEYCODE_J)    PORT_CHAR('j') PORT_CHAR('J')
		PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("K")  PORT_CODE(KEYCODE_K)    PORT_CHAR('k') PORT_CHAR('K')
		PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("L")  PORT_CODE(KEYCODE_L)    PORT_CHAR('l') PORT_CHAR('L')
		PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Z")  PORT_CODE(KEYCODE_Z)    PORT_CHAR('z') PORT_CHAR('Z')
		PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("X")  PORT_CODE(KEYCODE_X)    PORT_CHAR('x') PORT_CHAR('X')
		PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("C")  PORT_CODE(KEYCODE_C)    PORT_CHAR('c') PORT_CHAR('C')
		PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("V")  PORT_CODE(KEYCODE_V)    PORT_CHAR('v') PORT_CHAR('V')
		PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("B")  PORT_CODE(KEYCODE_B)    PORT_CHAR('b') PORT_CHAR('B')

	PORT_START("X3")
		PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("N")  PORT_CODE(KEYCODE_N)    PORT_CHAR('n') PORT_CHAR('N')
		PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("M")  PORT_CODE(KEYCODE_M)    PORT_CHAR('m') PORT_CHAR('M')
		PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("`")  PORT_CODE(KEYCODE_TILDE) PORT_CHAR('`') PORT_CHAR('~')
		PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("[")  PORT_CODE(KEYCODE_OPENBRACE) PORT_CHAR('[') PORT_CHAR(']')
		PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("\\") PORT_CODE(KEYCODE_BACKSLASH) PORT_CHAR('\\') PORT_CHAR('|')
		PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(";")  PORT_CODE(KEYCODE_COLON) PORT_CHAR(';') PORT_CHAR(':')
		PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("'")  PORT_CODE(KEYCODE_QUOTE) PORT_CHAR('\'') PORT_CHAR('"')
		PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("{")  PORT_CODE(KEYCODE_CLOSEBRACE) PORT_CHAR('{') PORT_CHAR('}')

	PORT_START("X4")
		PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("<")  PORT_CODE(KEYCODE_PGDN) PORT_CHAR('<') PORT_CHAR('>')
		PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(",")  PORT_CODE(KEYCODE_COMMA) PORT_CHAR(',')
		PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(".")  PORT_CODE(KEYCODE_STOP) PORT_CHAR('.')
		PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("/")  PORT_CODE(KEYCODE_SLASH) PORT_CHAR('/') PORT_CHAR('?')
		PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("1")  PORT_CODE(KEYCODE_1)    PORT_CHAR('1') PORT_CHAR('!')
		PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("2")  PORT_CODE(KEYCODE_2)    PORT_CHAR('2') PORT_CHAR('@')
		PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("3")  PORT_CODE(KEYCODE_3)    PORT_CHAR('3') PORT_CHAR('#')
		PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("4")  PORT_CODE(KEYCODE_4)    PORT_CHAR('4') PORT_CHAR('$')

	PORT_START("X5")
		PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("5")  PORT_CODE(KEYCODE_5)    PORT_CHAR('5') PORT_CHAR('%')
		PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("6")  PORT_CODE(KEYCODE_6)    PORT_CHAR('6') PORT_CHAR('^')
		PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("7")  PORT_CODE(KEYCODE_7)    PORT_CHAR('7') PORT_CHAR('&')
		PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("8")  PORT_CODE(KEYCODE_8)    PORT_CHAR('8') PORT_CHAR('*')
		PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("9")  PORT_CODE(KEYCODE_9)    PORT_CHAR('9') PORT_CHAR('(')
		PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("0")  PORT_CODE(KEYCODE_0)    PORT_CHAR('0') PORT_CHAR(')')
		PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("-")  PORT_CODE(KEYCODE_MINUS) PORT_CHAR('-') PORT_CHAR('_')
		PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("=")  PORT_CODE(KEYCODE_EQUALS) PORT_CHAR('=') PORT_CHAR('+')

	PORT_START("X6")
		PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("7pad")   PORT_CODE(KEYCODE_7_PAD)
		PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("8pad")   PORT_CODE(KEYCODE_8_PAD)
		PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("9pad")   PORT_CODE(KEYCODE_9_PAD)
		PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("X7")
		PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_UNUSED) // Does a HOME
		PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Home")   PORT_CODE(KEYCODE_HOME)
		PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("4pad")   PORT_CODE(KEYCODE_4_PAD)
		PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("5pad")   PORT_CODE(KEYCODE_5_PAD)
		PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("6pad")   PORT_CODE(KEYCODE_6_PAD)
		PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Backspace")  PORT_CODE(KEYCODE_BACKSPACE) PORT_CHAR(8)
		PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("X8")
		PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("1pad")   PORT_CODE(KEYCODE_1_PAD)
		PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("2pad")   PORT_CODE(KEYCODE_2_PAD)
		PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("3pad")   PORT_CODE(KEYCODE_3_PAD)
		PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Down")   PORT_CODE(KEYCODE_DOWN)
		PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Up") PORT_CODE(KEYCODE_UP)
		PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("0pad")   PORT_CODE(KEYCODE_0_PAD)

	PORT_START("X9")
		PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(".pad")   PORT_CODE(KEYCODE_DEL_PAD)
		PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("-pad")   PORT_CODE(KEYCODE_MINUS_PAD)
		PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Left")   PORT_CODE(KEYCODE_LEFT)

	PORT_START("X10")
		PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Right")  PORT_CODE(KEYCODE_RIGHT)
		PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Space")  PORT_CODE(KEYCODE_SPACE) PORT_CHAR(' ')
		PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("XMIT")   PORT_CODE(KEYCODE_ENTER) PORT_CHAR(13)
		PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_UNUSED)
		PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_UNUSED)

	// This row is scanned but nothing happens
	PORT_START("X11")
		PORT_BIT(0xff, IP_ACTIVE_LOW, IPT_UNUSED)

	// These probably not exist
	PORT_START("X12")
		PORT_BIT(0xff, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_START("X13")
		PORT_BIT(0xff, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_START("X14")
		PORT_BIT(0xff, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_START("X15")
		PORT_BIT(0xff, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("MODIFIERS")
		PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Capslock") PORT_CODE(KEYCODE_CAPSLOCK)
		PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("LCtrl") PORT_CODE(KEYCODE_LCONTROL) PORT_CHAR(UCHAR_SHIFT_2)
		PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("RCtrl") PORT_CODE(KEYCODE_RCONTROL)
		PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("LShift") PORT_CODE(KEYCODE_LSHIFT) PORT_CHAR(UCHAR_SHIFT_1)
		PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("RShift") PORT_CODE(KEYCODE_RSHIFT)
		PORT_BIT(0x83, IP_ACTIVE_LOW, IPT_UNUSED)

		// assumed to be dipswitches, purpose unknown, see code from 12D
	PORT_START("DIPS")
	PORT_DIPNAME( 0x01, 0x01, "Switch A") PORT_DIPLOCATION("SW1:1")
	PORT_DIPSETTING(    0x01, DEF_STR(Off))
	PORT_DIPSETTING(    0x00, DEF_STR(On))
	PORT_DIPNAME( 0x02, 0x02, "Switch B") PORT_DIPLOCATION("SW1:2")
	PORT_DIPSETTING(    0x02, DEF_STR(Off))
	PORT_DIPSETTING(    0x00, DEF_STR(On))
	PORT_BIT(0x3c, 0x2c, IPT_UNUSED) // this is required to sync keyboard and A-LOCK indicator
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_UNUSED) // unused
	PORT_DIPNAME( 0x80, 0x80, "Switch C") PORT_DIPLOCATION("SW1:3")
	PORT_DIPSETTING(    0x80, DEF_STR(Off))
	PORT_DIPSETTING(    0x00, DEF_STR(On))
INPUT_PORTS_END


void beehive_state::machine_reset()
{
}

/* This system appears to have inline attribute bytes of unknown meaning.
    Currently they are ignored. */
uint32_t beehive_state::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	uint16_t cursor_pos = (m_p_videoram[0xcaf] | (m_p_videoram[0xcb0] << 8)) & 0xfff;
	uint16_t p_linelist;
	uint8_t line_length;
	uint8_t y,ra,chr,gfx,inv;
	uint16_t sy=0,ma,x;

	for (y = 0; y < 25; y++)
	{
		p_linelist = 0x1af + y*3;
		line_length = m_p_videoram[p_linelist]+1;
		ma = (m_p_videoram[p_linelist+1] | (m_p_videoram[p_linelist+2] << 8)) & 0xfff;

		for (ra = 0; ra < 10; ra++)
		{
			uint16_t *p = &bitmap.pix16(sy++);
			uint8_t chars = 0;

			for (x = ma; x < ma + line_length; x++)
			{
				inv = gfx = chr = 0;
				if (y == 24) inv=0xff; // status line reverse video
				if (ra < 9)
				{
					if (x == cursor_pos) inv=0xff; // show cursor
					chr = m_p_videoram[x]; // get char in videoram
					gfx = m_p_chargen[(chr<<4) | ra ] ^ inv; // get dot pattern in chargen
				}

				if ((chars < 80) && (!BIT(chr, 7)))  // ignore attribute bytes
				{
					chars++;

					/* Display a scanline of a character */
					*p++ = BIT(gfx, 7);
					*p++ = BIT(gfx, 6);
					*p++ = BIT(gfx, 5);
					*p++ = BIT(gfx, 4);
					*p++ = BIT(gfx, 3);
					*p++ = BIT(gfx, 2);
					*p++ = BIT(gfx, 1);
					*p++ = BIT(gfx, 0);
				}
			}
		}
	}
	return 0;
}

MACHINE_CONFIG_START(beehive_state::beehive)
	/* basic machine hardware */
	MCFG_DEVICE_ADD("maincpu",I8085A, XTAL(4'000'000))
	MCFG_DEVICE_PROGRAM_MAP(beehive_mem)
	MCFG_DEVICE_IO_MAP(beehive_io)

	/* video hardware */
	MCFG_SCREEN_ADD_MONOCHROME("screen", RASTER, rgb_t::green())
	MCFG_SCREEN_REFRESH_RATE(50)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500)) /* not accurate */
	MCFG_SCREEN_UPDATE_DRIVER(beehive_state, screen_update)
	MCFG_SCREEN_SIZE(640, 250)
	MCFG_SCREEN_VISIBLE_AREA(0, 639, 0, 249)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_PALETTE_ADD_MONOCHROME("palette")
MACHINE_CONFIG_END

/* ROM definition */
ROM_START( beehive )
	ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASEFF )
	ROM_LOAD( "dm3270-1.rom", 0x0000, 0x0800, CRC(781bde32) SHA1(a3fe25baadd2bfc2b1791f509bb0f4960281ee32) )
	ROM_LOAD( "dm3270-2.rom", 0x0800, 0x0800, CRC(4d3476b7) SHA1(627ad42029ca6c8574cda8134d047d20515baf53) )
	ROM_LOAD( "dm3270-3.rom", 0x1000, 0x0800, CRC(dbf15833) SHA1(ae93117260a259236c50885c5cecead2aad9b3c4) )

	/* character generator not dumped, using the one from 'c10' for now */
	ROM_REGION( 0x2000, "chargen", 0 )
	ROM_LOAD( "c10_char.bin", 0x0000, 0x2000, BAD_DUMP CRC(cb530b6f) SHA1(95590bbb433db9c4317f535723b29516b9b9fcbf))
ROM_END

/* Driver */

//    YEAR  NAME     PARENT  COMPAT  MACHINE  INPUT    CLASS          INIT        COMPANY    FULLNAME  FLAGS
COMP( 1982, beehive, 0,      0,      beehive, beehive, beehive_state, empty_init, "BeeHive", "DM3270", MACHINE_NO_SOUND)
