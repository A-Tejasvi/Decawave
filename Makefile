CC=avr-gcc
# OBJCOPY=avr-objcopy
LIBS = -lm
CFLAGS=-Os -DF_CPU=16000000UL -mmcu=atmega328p -Iinclude
# # DEPS
# # PORT=/dev/ttyACM0

# CPPFLAGS = -Iinclude
# # CFLAGS = -Wall $(DEBUG)
# # CXXFLAGS = $(CFLAGS)

# # test.hex: test.elf
# # 	${OBJCOPY} -O ihex -R .eeprom test.elf test.hex

# # test.elf: test.o libdw1000.o libdw1000Spi.o
# # 	${CC} ${CFLAGS} -o test.elf test.o libdw1000.o libdw1000Spi.o
SRCDIR=src
OBJDIR=obj
INCDIR=include
objects = $(addprefix $(OBJDIR)/, libdw1000.o libdw1000Spi.o)


# $(OBJDIR)/dwlib.o: 
# 	$(CC) ${CFLAGS} -o $@ -c $(objects)
# libdw1000Spi.o: src/libdw1000Spi.c include/libdw1000Spi.h
# 	${CC} ${CFLAGS} -c src/libdw1000Spi.c

main: $(OBJDIR)/test.o $(objects)
	${CC} ${CFLAGS} -o $@ $(objects) $(OBJDIR)/test.o $(LIBS)

$(OBJDIR)/test.o: test.cpp test.h $(objects)
	${CC} ${CFLAGS} -o $@ -c test.cpp

$(OBJDIR)/libdw1000.o: $(SRCDIR)/libdw1000.c
	$(CC) $(CFLAGS) -o $(OBJDIR)/libdw1000.o -c $(SRCDIR)/libdw1000.c

$(OBJDIR)/libdw1000Spi.o: $(SRCDIR)/libdw1000Spi.c
	$(CC) $(CFLAGS) -o $(OBJDIR)/libdw1000Spi.o -c $(SRCDIR)/libdw1000Spi.c

.PHONY: cleanall
cleanall:
	rm obj/*.o
# install: test.hex
# 	avrdude -F -V -c arduino -p ATMEGA328P -P ${PORT} -b 115200 -U flash:w:test.hex