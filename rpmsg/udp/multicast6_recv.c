#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <linux/in.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <net/if.h>

int main(int argc, char **argv) {
   if(argc < 3) {
      fprintf(stderr, "Usage: %s mcast_addr port\n", argv[0]);
      exit(1);
   }

   int sock = socket(AF_INET6, SOCK_DGRAM, 0);
   if (sock < 0) {
      perror("socket");
      exit(1);
   }

   struct sockaddr_in6 addr;
   memset(&addr, 0, sizeof(addr));
   addr.sin6_family = AF_INET6;
   inet_pton(AF_INET6, argv[1], &addr.sin6_addr);
   addr.sin6_port = htons(atoi(argv[2]));

   if (bind(sock, (struct sockaddr *) &addr, sizeof(addr)) < 0) {        
      perror("bind");
      exit(1);
   }    

   struct ipv6_mreq mreq;
   mreq.ipv6mr_interface = 0;
   // mreq.ipv6mr_interface = if_nametoindex("enp0s25"); // or choose concrete interface
   inet_pton(AF_INET6, argv[1], &mreq.ipv6mr_multiaddr);
   if (setsockopt(sock, IPPROTO_IPV6, IPV6_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
      perror("setsockopt mreq");
      exit(1);
   }         

   for(;;) {
      struct sockaddr_in6 srcaddr;
      socklen_t addrlen = sizeof(srcaddr);
      char buf[1024];
      char ip[INET6_ADDRSTRLEN];
      int r = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *) &srcaddr, &addrlen);
      if (r <= 0) {
	 perror("recvfrom");
	 exit(1);
      }

      buf[r] = 0;
      inet_ntop(AF_INET6, &srcaddr.sin6_addr, ip, sizeof(ip));
      printf("[%s:%d]: '%s'\n", ip, addr.sin6_port, buf);
   }

   return 0;
}

