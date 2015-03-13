all: px4flow

px4flow: main.cpp
	g++ -I libs/mavlink/include/mavlink/v1.0 libs/serial_port.cpp libs/px4flow_interface.cpp main.cpp -o mavpx4flow.run -lpthread `pkg-config --cflags --libs opencv` -std=c++11

clean:
	 rm -rf *.o *.run

run-test:
	./mavpx4flow.run