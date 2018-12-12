#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <linux/in.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define PORT 12345

int main() {
   int sock = socket(AF_INET, SOCK_DGRAM, 0);
   if (sock < 0) {
      perror("socket");
      exit(1);
   }

   struct sockaddr_in addr;
   memset(&addr, 0, sizeof(addr));
   addr.sin_family = AF_INET;
   addr.sin_addr.s_addr = INADDR_BROADCAST;
   addr.sin_port = htons(PORT);
   socklen_t addrlen = sizeof(addr);

   if (bind(sock, (struct sockaddr *) &addr, sizeof(addr)) < 0) {        
      perror("bind");
      exit(1);
   }

   int enable = 1;
   if(setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &enable, sizeof(enable)) != 0) {
      perror("setsockopt");
      exit(1);
   }

   int i = 0;
   for(;;) {
      char buf[1024];
      int r = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *) &addr, &addrlen);
      if (r <= 0) {
	 perror("recvfrom");
	 exit(1);
      }
      buf[r] = 0;

      printf("%s: '%s'\n", inet_ntoa(addr.sin_addr), buf);
   }

   return 0;
}

