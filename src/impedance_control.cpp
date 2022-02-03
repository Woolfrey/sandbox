#include <Haiku.h>									// A simple poem in the form of a haiku
#include <Humanoid.h>									// Custom robot control class

int main(int argc, char *argv[])
{
	// Default for argc is 1, but I don't know why ¯\_(ツ)_/¯
	if(argc != 2)
	{
		std::cerr << "[ERROR] [IMPEDANCECONTROL] Path to urdf model required."
			<< " Usage: './impedance_control /path/to/model.urdf'" << std::endl;
		return 1;								// Close with error
	}
	else
	{
		std::string file = argv[1];						// Get the urdf model path
		Humanoid robot(file);							// Create object	

		std::cout << "[INFO] [IMPEDANCECONTROL] All done." << std::endl;
		
		return 0;								// No problems with main
	}
}
