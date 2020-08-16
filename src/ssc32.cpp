#include "lynxmotion_ssc32/ssc32.h"

#ifndef DEBUG
#define DEBUG 0
#endif

namespace lynxmotion_ssc32
{

//Constructor
SSC32::SSC32( ) :
	fd( -1 )
{
	unsigned int i;
	for( i = 0; i < SSC32::MAX_CHANNELS; i++ )
		first_instruction[i] = 0;
}

//Destructor
SSC32::~SSC32( )
{
	close_port( );
}

// Read function
int SSC32::read(int fd, char *buf, int size ) {
	return sp.Receive(buf, size);
}

// Write function
int SSC32::write( int fd, char *msg, int size ) {
       	return sp.Transmit(msg, size);
}

bool SSC32::open_port( const char *port, int baud )
{
	if (fd == 1) return true;
	char *portName = (char *)port;
	if (!sp.OpenOk(portName)) return false; 
	if (!sp.SetProtocol(baud, 8, ONESTOPBIT, NOPARITY)) return false;
	fd = 1;
	return true;
}

bool SSC32::is_connected( )
{
	if (fd == 1) return true;
	return false;
}

void SSC32::close_port( )
{
	sp.Close();
}

bool SSC32::send_message( const char *msg, int size )
{
	if( fd != -1 )
	{
		// tcflush( fd, TCIOFLUSH );

#if DEBUG
		printf( "INFO: [send_message] Sending message: " );
		for( unsigned int i = 0; i < strlen( msg ); i++ )
		{
			if( msg[i] == '\r' )
				printf( "<cr>" );
			else if( msg[i] == 27 )
				printf( "<esc>" );
			else
				printf( "%c", msg[i] );
		}
		printf( "\n" );
#endif

		if( write( fd, (char *)msg, size ) < 0 )
		{
#if DEBUG
			printf( "ERROR: [send_message] Failed to write to device\n" );
#endif
			return false;
		}

	}
	else
	{
#if DEBUG
		printf( "ERROR: [send_message] Device is not open\n" );
#endif
		return false;
	}

	return true;
}

unsigned int SSC32::recv_message( unsigned char *buf, unsigned int size )
{
	int bytes_read;
	int total_bytes = 0;

	while( total_bytes != size )
	{
		if( ( bytes_read = read( fd, (char *)(buf + total_bytes), 1 ) ) < 0 )
		{
#if DEBUG
			printf( "ERROR: [recv_message] Failed to read from device\n" );
#endif
			return total_bytes;
		}

		total_bytes += bytes_read;
	}

	return total_bytes;
}

bool SSC32::move_servo( struct ServoCommand cmd, int time )
{
	return move_servo( &cmd, 1, time );
}

bool SSC32::move_servo( struct ServoCommand cmd[], unsigned int n, int time )
{
	char msg[1024] = { 0 };
	char temp[32];
	int time_flag;
	unsigned int i;
	bool result;

	time_flag = 0;

	if( n > SSC32::MAX_CHANNELS )
	{
#if DEBUG
		printf( "ERROR: [move_servo] Invalid number of channels [%u]\n", n );
#endif
		return false;
	}

	for( i = 0; i < n; i++ )
	{
		if( cmd[i].ch > 31 )
		{
#if DEBUG
			printf( "ERROR: [move_servo] Invalid channel [%u]\n", cmd[i].ch );
#endif
			return false;
		}

		if( cmd[i].pw < SSC32::MIN_PULSE_WIDTH || cmd[i].pw > SSC32::MAX_PULSE_WIDTH )
		{
#if DEBUG
			printf( "ERROR: [move_servo] Invalid pulse width [%u]\n", cmd[i].pw );
#endif
			return false;
		}


		sprintf_s( temp, "#%u P%u ", cmd[i].ch, cmd[i].pw );

		strcat_s( msg, temp );

		if( first_instruction[cmd[i].ch] != 0 )
		{
			if( cmd[i].spd > 0 )
			{
				sprintf_s( temp, "S%d ", cmd[i].spd );
				strcat_s( msg, temp );
			}
		}
		else // this is the first instruction for this channel
			time_flag++;
	}

	// If time_flag is 0, then this is not the first instruction
	// for any channels to move the servo
	if( time_flag == 0 && time > 0 )
	{
		sprintf_s( temp, "T%d ", time );
		strcat_s( msg, temp );
	}

	strcat_s( msg, "\r" );

	result = send_message( msg, strlen( msg ) );

	// If the command was success, then the channels commanded
	// are not on their first instuction anymore.
	if( result )
		for( i = 0; i < n; i++ )
			first_instruction[cmd[i].ch] = 1;

	return result;
}

bool SSC32::cancel_command( )
{
	char msg[4];
	sprintf_s( msg, "%c \r", 27 );

	return send_message( msg, strlen( msg ) );
}

bool SSC32::pulse_offset( unsigned int ch, int value )
{
	return pulse_offset( &ch, &value, 1 );
}

bool SSC32::pulse_offset( unsigned int ch[], int value[], unsigned int n )
{
	char msg[1024] = { 0 };
	char temp[12];
	unsigned int i;

	if( n > SSC32::MAX_CHANNELS )
	{
#if DEBUG
		printf( "ERROR: [pulse_offset] Invalid number of channels [%u]\n", n );
#endif
		return false;
	}

	for( i = 0; i < n; i++ )
	{
		if( ch[i] > 31 )
		{
#if DEBUG
			printf( "ERROR: [pulse_offset] Invalid channel [%u]\n", ch[i] );
#endif
			return false;
		}

		if( value[i] < -100 || value[i] > 100 )
		{
#if DEBUG
			printf( "ERROR: [pulse_offset] Invalid offset value [%d]\n", value[i] );
#endif
			return false;
		}

		sprintf_s( temp, "#%u PO%d ", ch[i], value[i] );

		strcat_s( msg, temp );
	}

	strcat_s( msg, "\r" );

	return send_message( msg, strlen( msg ) );
}

bool SSC32::discrete_output( unsigned int ch, LogicLevel lvl )
{
	return discrete_output( &ch, &lvl, 1 );
}

bool SSC32::discrete_output( unsigned int ch[], LogicLevel lvl[], unsigned int n )
{
	char msg[1024] = { 0 };
	char temp[7];
	unsigned int i;

	if( n > SSC32::MAX_CHANNELS )
	{
#if DEBUG
		printf( "ERROR: [discrete_output] Invalid number of channels [%u]\n", n );
#endif
		return false;
	}

	for( i = 0; i < n; i++ )
	{
		if( ch[i] > 31 )
		{
#if DEBUG
			printf( "ERROR: [discrete_output] Invalid servo channel [%u]\n", ch[i] );
#endif
			return false;
		}

		sprintf_s( temp, "#%u %c ", ch[i], ( lvl[i] == High ) ? 'H' : 'L' );

		strcat_s( msg, temp );
	}

	strcat_s( msg, "\r" );

	return send_message( msg, strlen( msg ) );
}

bool SSC32::byte_output( unsigned int bank, unsigned int value )
{
	char msg[10];

	if( bank > 3 )
	{
#if DEBUG
		printf( "ERROR: [byte_output] Invalid bank [%u]\n", bank );
#endif
		return false;
	}

	if( value > 255 )
	{
#if DEBUG
		printf( "ERROR: [byte_output] Invalid value [%u]\n", value );
#endif
		return false;
	}

	sprintf_s( msg, "#%d:%d \r", bank, value );

	return send_message( msg, strlen( msg ) );
}

bool SSC32::query_movement_status( )
{
	unsigned char buffer;
	//int bytes_read = 0;
	const char *msg = "Q \r";

	if( !send_message( msg, strlen( msg ) ) )
	{
#if DEBUG
		printf( "ERROR: [query_movement_status] Failed to send message\n" );
#endif
		return false;
	}

	// There is a delay of at least 50uS to 5mS, so sleep for 5ms.
	Sleep( 10 );

	// Continue reading from controller until a response is received
	if( recv_message( &buffer, 1 ) != 1 )
	{
#if DEBUG
		printf( "ERROR: [query_movement_status] Failed to receive message\n" );
#endif
		return false;
	}

	// Check response value
	if( buffer == '+' )
		return true;

	return false;
}

int SSC32::query_pulse_width( unsigned int ch )
{
	unsigned char buffer;
	int bytes_read = 0;
	char msg[7];

	// Check if the servo channel is valid
	if( ch > 31 )
	{
#if DEBUG
		printf( "ERROR: [query_pulse_width] Invalid servo channel [%u]\n", ch );
#endif
		return false;
	}

	sprintf_s( msg, "QP%d \r", ch );

	if( !send_message( msg, strlen( msg ) ) )
        {
#if DEBUG
                printf( "ERROR: [query_pulse_width] Failed to send message\n" );
#endif
                return false;
        }

	// It can take up to 5ms before the controller responds, so sleep for 5ms.
	Sleep( 5 );

	if( recv_message( &buffer, 1 ) != 1 )
	{
#if DEBUG
		printf( "ERROR: [query_pulse_width] Failed to receive message\n" );
#endif
		return false;
	}

	return ( 10 * ( int )buffer );
}

bool SSC32::read_digital_inputs( Inputs inputs[], unsigned int outputs[], unsigned int n )
{
	unsigned char buffer[8];
	int bytes_read = 0;
	int total_bytes = 0;
	char msg[255] = { 0 };
	int i;

	// SSC-32U documentation states that only up to 8 values can be read at once
	if ( n > 8 )
	{
		printf( "WARNING: reading digital inputs -- n must not be greater than 8\n" );
		n = 8;
	}

	for( i = 0; i < n; i++ )
	{
		switch( inputs[i] )
		{
			case PinA:  strcat_s( msg, "A " );  break;
			case PinAL: strcat_s( msg, "AL " ); break;
			case PinB:  strcat_s( msg, "B " );  break;
			case PinBL: strcat_s( msg, "BL " ); break;
			case PinC:  strcat_s( msg, "C " );  break;
			case PinCL: strcat_s( msg, "CL " ); break;
			case PinD:  strcat_s( msg, "D " );  break;
			case PinDL: strcat_s( msg, "DL " ); break;
			case PinE:  strcat_s( msg, "E " );  break;
			case PinEL: strcat_s( msg, "EL " ); break;
			case PinF:  strcat_s( msg, "F " );  break;
			case PinFL: strcat_s( msg, "FL " ); break;
			default:
#if DEBUG
				printf( "WARNING: [read_digital_inputs] Unrecognized input value [%d]\n", inputs[i] );
#endif
				break;
		}
	}

	strcat_s( msg, "\r" );

	if( !send_message( msg, strlen( msg ) ) )
        {
#if DEBUG
		printf( "ERROR: [read_digital_inputs] Failed to send message\n" );
#endif
		return false;
        }

	if( recv_message( buffer, n ) != n )
	{
#if DEBUG
		printf( "ERROR: [read_digital_inputs] Failed to receive message\n" );
#endif
		return false;
	}

	for( i = 0; i < n; i++ )
		outputs[i] = buffer[i] - '0';

	return true;
}

bool SSC32::read_analog_inputs( Inputs inputs[], float outputs[], unsigned int n )
{
	unsigned char buffer[8];
	int bytes_read = 0;
	int total_bytes = 0;
	char msg[255] = { 0 };
	int i;

	if( n > 8 )
	{
		printf( "WARNING: reading analog inputs -- n must not be greater than 8\n" );
		n = 8;
	}

	for( i = 0; i < n; i++ )
	{
		switch( inputs[i] )
		{
			case PinA: strcat_s( msg, "VA " ); break;
			case PinB: strcat_s( msg, "VB " ); break;
			case PinC: strcat_s( msg, "VC " ); break;
			case PinD: strcat_s( msg, "VD " ); break;
			case PinE: strcat_s( msg, "VE " ); break;
			case PinF: strcat_s( msg, "VF " ); break;
			case PinG: strcat_s( msg, "VG " ); break;
			case PinH: strcat_s( msg, "VH " ); break;
			default:
#if DEBUG
				printf( "WARNING: [read_analog_inputs] Unrecognized input value [%d]\n", inputs[i] );
#endif
				break;
		}
	}

	strcat_s( msg, "\r" );

	if( !send_message( msg, strlen( msg ) ) )
        {
#if DEBUG
		printf( "ERROR: [read_analog_inputs] Failed to send message\n" );
#endif
		return false;
        }

	if( recv_message( buffer, n ) != n )
	{
#if DEBUG
		printf( "ERROR: [read_analog_inputs] Failed to receive message\n" );
#endif
		return false;
	}

	for( i = 0; i < n; i++ )
		outputs[i] = 5.0 * buffer[i] / 256.0;

	return true;
}

std::string SSC32::get_version( )
{
	char data[255];
	int bytes_read;
	int total_bytes;
	int i;
	std::string version;
	const char *msg = "VER\r";

	total_bytes = 0;

	if( !send_message( msg, strlen( msg ) ) )
	{
#if DEBUG
		printf( "ERROR: [get_version] Failed to send message\n" );
#endif
		return "error";
	}

	Sleep( 10 );

#if DEBUG
	printf( "INFO: [get_version] Reading response\n" );
#endif

	while( ( bytes_read = read( fd, data + total_bytes, 1 ) ) > 0 )
		total_bytes += bytes_read;

#if DEBUG
	printf( "INFO: [get_version] Data: " );
	for( i = 0; i < total_bytes; i++ )
	{
		if( data[i] == '\r' )
			printf( "<cr>" );
		else if( data[i] == '\n' )
			printf( "<nl>" );
		else
			printf( "%c", data[i] );
	}
	printf( "\n" );
#endif

	if( bytes_read < 0 )
	{
#if DEBUG
		printf( "ERROR: [get_version] Failed to read from device\n" );
#endif
	}
	else if( total_bytes > 0 )
	{
#if DEBUG
		printf( "Read %d bytes\n", total_bytes );
#endif

		if( data[total_bytes - 1] == '\r' )
			data[total_bytes - 1] = '\0';
		else
		{
#if DEBUG
			printf( "WARNING: [get_version] Timeout while reading\n" );
#endif
			data[total_bytes] = '\0';
		}

		i = total_bytes - 2;

		while( i >= 0 && data[i] != '\r' )
			i--;

		version = data + i + 1;
	}
	else
	{
#if DEBUG
		printf( "WARNING: [get_version] Timeout while reading\n" );
#endif
	}

	return version;
}

} // namespace
