OBJS=main.o i2c_freebsd.o bme280.o dht12.o

all: i2c_test
	
%.o: %.c
	$(CC) -c $< -o $@

i2c_test: $(OBJS)
	$(CC) $(CFLAGS) -o $@ $(OBJS)
