# MicroPython I2C LCD Library for Smart Garden
# 20x4 I2C LCD Display Controller

class I2CLCD:
    def __init__(self, i2c, cols=20, rows=4, address=0x27):
        self.i2c = i2c
        self.cols = cols
        self.rows = rows
        self.address = address
        
        # LCD Commands
        self.LCD_CLEARDISPLAY = 0x01
        self.LCD_RETURNHOME = 0x02
        self.LCD_ENTRYMODE = 0x04
        self.LCD_DISPLAYCONTROL = 0x08
        self.LCD_CURSORSHIFT = 0x10
        self.LCD_FUNCTIONSET = 0x20
        self.LCD_SETCGRAMADDR = 0x40
        self.LCD_SETDDRAMADDR = 0x80
        
        # Display entry mode
        self.LCD_ENTRYRIGHT = 0x00
        self.LCD_ENTRYLEFT = 0x02
        self.LCD_ENTRYSHIFTINCREMENT = 0x01
        self.LCD_ENTRYSHIFTDECREMENT = 0x00
        
        # Display control
        self.LCD_DISPLAYON = 0x04
        self.LCD_DISPLAYOFF = 0x00
        self.LCD_CURSORON = 0x02
        self.LCD_CURSOROFF = 0x00
        self.LCD_BLINKON = 0x01
        self.LCD_BLINKOFF = 0x00
        
        # Function set
        self.LCD_8BITMODE = 0x10
        self.LCD_4BITMODE = 0x00
        self.LCD_2LINE = 0x08
        self.LCD_1LINE = 0x00
        self.LCD_5x10DOTS = 0x04
        self.LCD_5x8DOTS = 0x00
        
        # Backlight control
        self.LCD_BACKLIGHT = 0x08
        self.LCD_NOBACKLIGHT = 0x00
        
        self._En = 0b00000100  # Enable bit
        self._Rs = 0b00000001  # Register select bit
        
        self._backlight = self.LCD_BACKLIGHT
        
        self.init()
    
    def init(self):
        """Initialize LCD display"""
        try:
            # Initial configuration for 4-bit mode
            self._write_byte(0x33, self.LCD_CMD)  # 110011 Initialize
            self._write_byte(0x32, self.LCD_CMD)  # 110010 Initialize
            self._write_byte(0x06, self.LCD_CMD)  # 000110 Cursor move direction
            self._write_byte(0x0C, self.LCD_CMD)  # 001100 Display on,Cursor off, Blink off
            self._write_byte(0x28, self.LCD_CMD)  # 101000 Data length, Number of lines, Font size
            self.clear()
            print("✅ LCD initialized successfully")
        except Exception as e:
            print(f"❌ LCD initialization failed: {e}")
    
    def clear(self):
        """Clear display and return home"""
        self._write_byte(self.LCD_CLEARDISPLAY, self.LCD_CMD)
        self._write_byte(self.LCD_RETURNHOME, self.LCD_CMD)
    
    def home(self):
        """Return cursor to home position"""
        self._write_byte(self.LCD_RETURNHOME, self.LCD_CMD)
    
    def text(self, string, row=0, col=0):
        """Display text at specified position"""
        if len(string) > self.cols:
            string = string[:self.cols]
        
        # Set cursor position
        row_offsets = [0x00, 0x40, 0x14, 0x54]
        if row < len(row_offsets):
            self._write_byte(self.LCD_SETDDRAMADDR | (col + row_offsets[row]), self.LCD_CMD)
        
        # Display string
        for char in string:
            self._write_byte(ord(char), self.LCD_CHR)
    
    def backlight(self, state):
        """Turn backlight on or off"""
        if state:
            self._backlight = self.LCD_BACKLIGHT
        else:
            self._backlight = self.LCD_NOBACKLIGHT
    
    def cursor(self, row, col):
        """Move cursor to position"""
        row_offsets = [0x00, 0x40, 0x14, 0x54]
        if row < len(row_offsets):
            self._write_byte(self.LCD_SETDDRAMADDR | (col + row_offsets[row]), self.LCD_CMD)
    
    def display(self, state):
        """Turn display on or off"""
        if state:
            self._write_byte(self.LCD_DISPLAYCONTROL | self.LCD_DISPLAYON | self._backlight, self.LCD_CMD)
        else:
            self._write_byte(self.LCD_DISPLAYCONTROL | self.LCD_DISPLAYOFF | self._backlight, self.LCD_CMD)
    
    def _write_byte(self, data, mode):
        """Write byte to LCD"""
        try:
            # High nibble
            highnib = data & 0xF0
            self._write_4bits(highnib | mode)
            
            # Low nibble
            lownib = (data << 4) & 0xF0
            self._write_4bits(lownib | mode)
        except Exception as e:
            print(f"❌ LCD write error: {e}")
    
    def _write_4bits(self, data):
        """Write 4 bits to LCD"""
        try:
            self.i2c.writeto(self.address, bytes([data | self._backlight]))
            self._strobe(data)
        except Exception as e:
            print(f"❌ LCD 4-bit write error: {e}")
    
    def _strobe(self, data):
        """Strobe LCD enable pin"""
        try:
            # Enable pulse
            self.i2c.writeto(self.address, bytes([data | self._En | self._backlight]))
            time.sleep_us(1)
            
            # Disable pulse
            self.i2c.writeto(self.address, bytes([data & ~self._En | self._backlight]))
            time.sleep_us(50)
        except Exception as e:
            print(f"❌ LCD strobe error: {e}")
    
    def LCD_CMD(self):
        """Command mode constant"""
        return 0
    
    def LCD_CHR(self):
        """Character mode constant"""
        return 1