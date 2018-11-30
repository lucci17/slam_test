#include <uWS/uWS.h>
#include <iostream>
#include <signal.h>
#include <thread>
using namespace uWS;

bool main_thread_exit = false;
void ctrl_c_handler( int32_t sig )
{
	std::cout << "ctrl+c pressed" << std::endl;
	main_thread_exit = true;
}

int32_t init_singals()
{
	if( signal( SIGINT, ctrl_c_handler ) == SIG_ERR ) 
		return -1;
    
	return 0;
}

int main() 
{
    Hub h;
    std::string response = "Hello!";
	
	init_singals();

    h.onMessage([](WebSocket<SERVER> *ws, char *message, size_t length, OpCode opCode) 
	{
		if( opCode == uWS::TEXT)
			std::cout << std::string(message, length) << std::endl;
    });

	std::thread t(
		[&h]()
		{
			if( h.listen(3000) )
				h.run();
			
			std::cout << "thread exit" << std::endl;
		}
	);
	
    while( !main_thread_exit )
		usleep( 10000 );
	
	std::cout << "close server" << std::endl;
	h.getDefaultGroup<SERVER>().close();
	
	std::cout << "thread join" << std::endl;
	t.join();
	
	std::cout << "main thread exit" << std::endl;
	return 0;
}