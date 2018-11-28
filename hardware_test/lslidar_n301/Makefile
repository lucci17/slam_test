EXTRA_FLAGS := -std=gnu++11
OBJS := n301_decoder.o n301_driver.o udp_socket_client.o n301_test_main.o
TARGET:=test


all: $(OBJS)
	g++ $(OBJS) -o $(TARGET) $(EXTRA_FLAGS)

n301_decoder.o: n301_decoder.cpp
	g++ -c n301_decoder.cpp -o n301_decoder.o $(EXTRA_FLAGS)
n301_driver.o: n301_driver.cpp
	g++ -c n301_driver.cpp -o n301_driver.o $(EXTRA_FLAGS)
udp_socket_client.o: udp_socket_client.cpp
	g++ -c udp_socket_client.cpp -o udp_socket_client.o $(EXTRA_FLAGS)
n301_test_main.o: n301_test_main.cpp
	g++ -c n301_test_main.cpp -o n301_test_main.o $(EXTRA_FLAGS)
	
clean:
	rm -f *.o
	rm -f $(TARGET)
	
