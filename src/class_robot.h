#ifndef __ROBOT_H__
#define __ROBOT_H__

#include "struct_qnu.h"
#include "struct_nmpc.h"

class robot
{

private:

	char *hostname, *configfile;
	unsigned int port;
	float *poshead;
	int sockfd;

	// Class cannot be copied:
	robot(const robot&);
	robot& operator=(const robot&);

public:

	robot();
	~robot();

	int tcp_connect();
	int Nav2(const char*);
	void set_host(char*);
	void set_port(int);
	void set_configfile(char*);
	void update_poshead(qnu*, const nmpc&);
	int Nav2_v(float*, float*) ;
	char *conffile();

};

#endif
