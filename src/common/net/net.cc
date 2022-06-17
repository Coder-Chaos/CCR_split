#include "common.hpp"
#include "net.hpp"

namespace common
{
    Net::Net(const protocal_type &p_type, const type &type, const char *serv_ip_addr, const int &port) : p_type_{p_type}, type_{type}, server_ip_addr_{serv_ip_addr}, port_{port}
    {
        /* socket options */
        int old_option = 0;
        int new_option = 0;

        // check the type
        if (p_type_ == TCP)
        {
            /* code */
            sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
        }
        else if (p_type_ == UDP)
        {
            /* code */
            sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        }

        // set the address
        address_.sin_family = AF_INET;
        address_.sin_port = htons(port_);
        if (type_ == CLIENT)
        {
            // set dest ip address
            address_.sin_addr.s_addr = inet_addr(server_ip_addr_);
        }
        else if (type_ == SERVER)
        {
            // bind local ip address
            address_.sin_addr.s_addr = INADDR_ANY;
        }

        // bind if server
        if (type_ == SERVER)
        {
            int ret = bind(sockfd_, (struct sockaddr *)&address_, sizeof(address_));
            if (ret < 0)
            {
                /* code */
                perror("\rbind fail!\n");
                close(sockfd_);
                exit(-1);
            }
            else
            {
                old_option = fcntl(sockfd_, F_GETFL);
                new_option = old_option | O_NONBLOCK;
                fcntl(sockfd_, F_SETFL, new_option);

                /*监听连接请求--监听队列长度为5*/  
                listen(sockfd_,5);  
                
                socklen_t sin_size=sizeof(struct sockaddr_in);  
                printf("accept port:%d\n", port_);
                /*等待客户端连接请求到达*/  
                while((sockfd_client_=accept(sockfd_,(struct sockaddr *)&client_ip_addr_,&sin_size))<0) 
                //if((sockfd_client_=accept(sockfd_,(struct sockaddr *)&client_ip_addr_,&sin_size))<0) 
                {  
                    //printf("accept error\n");
                    usleep(1000); 
                    //if(port_ == 4001 || port_ == 9001)
                    //    exit(-1);
                        //break; 
                    //exit(-1);  
                }

                //printf("accept client %s\n",inet_ntoa(client_ip_addr_.sin_addr));   

                /*接收客户端的数据并将其发送给客户端--recv返回接收到的字节数，send返回发送的字节数*/  
                /*while((len=recv(sockfd_,buf,BUFSIZ,0))>0)  
                {  
                    buf[len]='\0';  
                    printf("Welcome Client!\nReceived:%s\n",buf);  
                    if(send(sockfd_,buf,len,0)<0)  
                    {  
                        perror("write");  
                        exit(-1);  
                    }  
                }*/
                printf("\rnetwork server created !\n");
                printf("\rserver address:%s, port:%d\n", server_ip_addr_, port_);
            }
        }
        else
        {
            printf("\rnetwork client created, port:%d.\n", port_);
        }
    }

    int Net::UdpSendtoServer(const void *buf, int len)
    {
        return sendto(sockfd_, buf, len, 0, (struct sockaddr *)&address_, sizeof(address_));
    }

    int Net::TcpRecvfromClient(void *buf, int len)
    {
        //printf("Recieve start\n");
        socklen_t sin_size = sizeof(struct sockaddr_in);
        //printf("Recieve start %d\n", recv(sockfd_client_, buf, len, 0));
        int nb = recv(sockfd_client_, buf, len, 0);
        //int nb = recvfrom(sockfd_client_, buf, len, 0, (struct sockaddr *)&address_, &sin_size);
        //printf("Recieve start %d\n", nb);
        /*while(nb == 0){
            while(accept(sockfd_,(struct sockaddr *)&client_ip_addr_,
                &sin_size) < 0){
                usleep(100);
            }
            printf("Recieve wait\n");
            recv(sockfd_client_, buf, len, 0);
            usleep(100); 
        }*/
        //printf("Recieve done\n");
        return nb;
    }

    int Net::TcpSendtoClient(const void *buf, int len)
    {
        //printf("Send start\n");
        socklen_t sin_size = sizeof(struct sockaddr_in);
        int nb = send(sockfd_client_, buf, len, 0);
        while(nb == 0){
            while(accept(sockfd_,(struct sockaddr *)&client_ip_addr_,
                &sin_size) < 0){
                usleep(100);
            }
            printf("Send wait\n");
            send(sockfd_client_, buf, len, 0);
            usleep(100); 
        }
        return nb;
    }

    void Net::closePort()
    {
      close(sockfd_);
    }

} // namespace common