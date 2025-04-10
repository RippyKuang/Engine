#include <arpa/inet.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/epoll.h>
#include <unistd.h>
#include <string>
#include <thread>
#include <iostream>
#include <functional>
#include <memory>

#define MAX_SIZE 1024
#define MAX_CONNECTION 36

namespace Engine
{
    template <typename T>
    class Publisher
    {
    private:
        std::string topic;
        struct epoll_event all[MAX_SIZE];
        struct sockaddr_in serv_addr;
        struct sockaddr_in client_addr;
        socklen_t cli_len = sizeof(client_addr);
        int lfd;
        int epfd;
        std::thread worker;
        void run()
        {
            while (1)
            {
                int ret = epoll_wait(epfd, all, sizeof(all) / sizeof(all[0]), -1);
                for (int i = 0; i < ret; ++i)
                {
                    int fd = all[i].data.fd;
                    if (fd == lfd)
                    {
                        int cfd = accept(lfd, (struct sockaddr *)&client_addr, &cli_len);
                        if (cfd == -1)
                            perror("> Accept Error");

                        int flag = fcntl(cfd, F_GETFL);
                        flag |= O_NONBLOCK;
                        fcntl(cfd, F_SETFL, flag);
                        struct epoll_event temp;
                        temp.events = EPOLLOUT;
                        temp.data.fd = cfd;
                        epoll_ctl(epfd, EPOLL_CTL_ADD, cfd, &temp);
                        char ip[64] = {0};
                        printf("> New Client [%s:%d] => [%d]\n",
                               inet_ntop(AF_INET, &client_addr.sin_addr.s_addr, ip, sizeof(ip)),
                               ntohs(client_addr.sin_port), cfd);
                    }
                }
            }
        }

    public:
        Publisher(std::string topic_name) : topic(topic_name)
        {
            std::hash<std::string> hasher;
            int port = hasher(topic_name) % 10000 + 10000;
            std::cout << port << std::endl;
            socklen_t serv_len = sizeof(serv_addr);

            lfd = socket(AF_INET, SOCK_STREAM, 0);
            memset(&serv_addr, 0, sizeof(serv_addr));
            serv_addr.sin_family = AF_INET;
            serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
            serv_addr.sin_port = htons(port);
            bind(lfd, (struct sockaddr *)&serv_addr, serv_len);
            char ip[64] = {0};
            printf("> New server [%s:%d] => [%d]\n",
                   inet_ntop(AF_INET, &serv_addr.sin_addr.s_addr, ip, sizeof(ip)),
                   ntohs(serv_addr.sin_port), lfd);
            listen(lfd, MAX_CONNECTION);

            epfd = epoll_create(1024);
            struct epoll_event ev;
            ev.events = EPOLLIN;
            ev.data.fd = lfd;
            epoll_ctl(epfd, EPOLL_CTL_ADD, lfd, &ev);
            worker = std::thread([this]
                                 { run(); });
        }

        void publish(T &data)
        {
            int ret = epoll_wait(epfd, all, sizeof(all) / sizeof(all[0]), 0);
            for (int i = 0; i < ret; i++)
            {
                if (all[i].events & EPOLLOUT)
                {
                    int sockfd = all[i].data.fd;
                    write(sockfd, &data, sizeof(data));
                }
            }
        }
        ~Publisher()
        {
            close(lfd);
        }
    };

    template <typename T>
    class Subscription
    {
    private:
        std::string topic;
        struct epoll_event all[MAX_SIZE];
        struct sockaddr_in client_addr;

        int lfd;
        int epfd;
        std::thread worker;
        std::function<void(const std::shared_ptr<T>)> cb;
        void run()
        {
            while (1)
            {
                int ret = epoll_wait(epfd, all, sizeof(all) / sizeof(all[0]), -1);
                for (int i = 0; i < ret; ++i)
                {
                    int fd = all[i].data.fd;
                    if (all[i].events & EPOLLIN)
                    {
                        T data;
                        int len = read(fd, &data, sizeof(data));
                        if (len > 0)
                        {
                            std::shared_ptr<T> pdata = std::make_shared<T>(data);
                            this->cb(pdata);
                        }
                    }
                }
            }
        }

    public:
        Subscription(std::string topic_name, std::function<void(const std::shared_ptr<T>)> cb) : topic(topic_name), cb(cb)
        {
            std::hash<std::string> hasher;
            int port = hasher(topic_name) % 10000 + 10000;

            socklen_t client_len = sizeof(client_addr);

            lfd = socket(AF_INET, SOCK_STREAM, 0);
            memset(&client_addr, 0, sizeof(client_addr));
            client_addr.sin_family = AF_INET;
            client_addr.sin_addr.s_addr = htonl(INADDR_ANY);
            client_addr.sin_port = htons(port);

            int ret = connect(lfd, (struct sockaddr *)&client_addr, client_len);
            if (ret == -1)
                perror("connect failed");
            epfd = epoll_create(1024);
            struct epoll_event ev;
            ev.events = EPOLLIN;
            ev.data.fd = lfd;
            epoll_ctl(epfd, EPOLL_CTL_ADD, lfd, &ev);
            worker = std::thread([this]
                                 { run(); });
        }
        ~Subscription()
        {
            close(lfd);
        }
    };

}