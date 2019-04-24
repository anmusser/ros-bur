#include <bur/plctypes.h>

#ifdef _DEFAULT_INCLUDES
#include <AsDefault.h>
#endif

class Hardware
{	 
	public:
	Hardware()
	{
		// Initializing all variables (otherwise they can have random values)
		_programStartTime = clock_ms();
		_step = 0;
		_ident = 0;
		_recvData = 0;
		_lastError = 0;
		_errorFlag = false;
		_recvTimeout = 0;
		brsmemset(reinterpret_cast<UDINT>(&_tcpOpen),0,sizeof(_tcpOpen));
		brsmemset(reinterpret_cast<UDINT>(&_tcpClient),0,sizeof(_tcpClient));
		brsmemset(reinterpret_cast<UDINT>(&_tcpRecv),0,sizeof(_tcpRecv));
		brsmemset(reinterpret_cast<UDINT>(&_tcpSend),0,sizeof(_tcpSend));
		brsmemset(reinterpret_cast<UDINT>(&_tcpClose),0,sizeof(_tcpClose));
	}
		 
	// any initialization code necessary to use the serial port
	void init()
	{		
		// we need to stay in this method until we've successfully connected, hence the while loop
		while (_step < 3) // STEP_CONNECTED
		{
			switch (_step)
			{
				case 0: // STEP_START
					initData();
					_step = 1; // STEP_OPEN_SOCKET
					break;
					
				/* opens ethernet interface for use with SSL */
				case 1: // STEP_OPEN_SOCKET
					openSocket();
					break;
			
				/* connects to the server */
				case 2: // STEP_CONNECT_TO_SERVER
					connectToServer();
					break;
				
				/* holding state once we're connected */
				case 3: // STEP_CONNECTED					
					break;
			}
		}				
	}
	
	// read a byte from the serial port. -1 = failure
	int read()
	{
		return receiveData();
	}
	
	// write data to the connection to ROS
	void write(USINT* data, int length)
	{
		sendData(data, length);
	}
	
	// returns milliseconds since start of program
	unsigned long time()
	{
		return (long) DiffT(clock_ms(),_programStartTime);
	}
	
	UDINT getLoopCounter()
	{
		return _loopCounter;
	}
	
	private:
	
	void initData()
	{
		/***** internal parameter assignments for all FBs *****/
		// TCP open FB
		_tcpOpen.pIfAddr 	= 0;
		_tcpOpen.port 		= 0;
		_tcpOpen.options 	= tcpOPT_REUSEADDR;	
		
		// TCP client FB
		_tcpClient.portserv = 11411;
		_tcpClient.pServer  = (UDINT) "192.168.56.101";
		
		// TCP receive FB
		_tcpRecv.pData   = (UDINT) &(_recvData);
		_tcpRecv.datamax = sizeof(_recvData);
		_tcpRecv.flags   = 0;
		
		// TCP send FB
		_tcpSend.flags   = 0;
	}	
	
	/* Opens a socket with the given port. */
	void openSocket()
	{
		_tcpOpen.enable = 1;	

		TcpOpen(&_tcpOpen);
						
		if (_tcpOpen.status == ERR_OK)
		{
			_step 	  = 2; // STEP_CONNECT_TO_SERVER
			_ident 	  = _tcpOpen.ident;
		}
		else if (_tcpOpen.status != ERR_FUB_BUSY )
		{
			_step	  = 500; // STEP_ERROR
			_lastError = _tcpOpen.status;
		}
	}
	
	/* Connects to the given server. */
	void connectToServer()
	{
		_tcpClient.enable   = 1;
		_tcpClient.ident 	  = _ident;
		TcpClient(&_tcpClient);
			
		if (_tcpClient.status == ERR_OK)
		{
			_step = 3; // STEP_CONNECTED
		}
		else if (_tcpClient.status != ERR_FUB_BUSY)
		{
			_errorFlag = true;
			_step 	  = 6; // STEP_CLOSE_SOCKET
			_lastError = _tcpClient.status;
		}
	}
	
	/* Receives data from server. */
	INT receiveData()
	{
		_tcpRecv.enable  = 1;
		_tcpRecv.ident   = _ident;
		while ((_tcpRecv.status == ERR_FUB_ENABLE_FALSE) || (_tcpRecv.status == ERR_FUB_BUSY))
		{
			TcpRecv(&_tcpRecv);
		}

		if (_tcpRecv.status == ERR_OK)
		{
			_recvTimeout = 0;
			_tcpRecv.enable = false;
			TcpRecv(&_tcpRecv);
			return _recvData;		
		}
		else if (_tcpRecv.status == tcpERR_NO_DATA) 					/* no data received - try again */
		{
			_recvTimeout 	   = _recvTimeout + 1;
			_tcpRecv.enable = false;
			TcpRecv(&_tcpRecv);
			return (-1 * _recvData);
		}
		else if (_tcpRecv.status == ERR_FUB_BUSY) 
		{
			return -1;
		}
		else // we have an error
		{
			_errorFlag = true;
			_step 	  = 6; // STEP_CLOSE_SOCKET
			_lastError = _tcpSend.status;
			_tcpRecv.enable = false;
			TcpRecv(&_tcpRecv);
			return -1;
		}
	}
	
	/* Sends data to the server */
	void sendData(USINT* data, int length)
	{	
		_tcpSend.enable  = 1;	
		_tcpSend.ident   = _ident;
		_tcpSend.pData   = (UDINT) data;
		_tcpSend.datalen = (UDINT) length;
		TcpSend(&_tcpSend);
		while ((_tcpSend.status == ERR_FUB_BUSY) || (_tcpSend.status == ERR_FUB_ENABLE_FALSE))
		{
			TcpSend(&_tcpSend);
		}
				
		if (_tcpSend.status == ERR_OK)
		{
			_tcpSend.enable = false;
			TcpSend(&_tcpSend);
			_step = 5; // STEP_RECEIVE_DATA
			return;
		}
		else if (_tcpSend.status != ERR_FUB_BUSY)
		{
			_tcpSend.enable = false;
			TcpSend(&_tcpSend);
			_errorFlag = true;
			_step	  = 6; // STEP_CLOSE_SOCKET
			_lastError = _tcpSend.status;
			return;
		} 
	}
	
	UDINT _programStartTime;
	UINT _step;
	UDINT _ident;
	USINT _recvData;
	UINT _lastError;
	BOOL _errorFlag;
	UDINT _recvTimeout;
	UDINT _loopCounter;
	UDINT _recvCounter;
	UDINT _sendCounter;
	TcpOpen_typ _tcpOpen;
	TcpClient_typ _tcpClient;
	TcpRecv_typ _tcpRecv;
	TcpSend_typ _tcpSend;
	TcpClose_typ _tcpClose;
};