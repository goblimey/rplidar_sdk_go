.PHONY: clean

CFLAGS += -I../include -DDEBUG -g

TARGET=DONE

$(TARGET): ../lib/librplidar_sdk.a proxy_driver.o driver.o cdriver.o
	go build .

%.o: %.cpp
	g++ -O2 $(CFLAGS) -o $@ -c $^

clean:
	rm -f *.o *.so *.a $(TARGET)
