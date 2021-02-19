package server_main

import (
	"go_simple_server/pkg/server-http"
	"go_simple_server/pkg/server-tcp"
	"log"
	"net"
	"net/http"
)

func SetupServers(addr string, portTcp string, portHttp string){
	go setupTcpServer(addr, portTcp)
	go setupHttpServer(addr, portHttp)
}

func setupTcpServer(host string, port string){
	addr := host + ":" + port

	listener, err := net.Listen("tcp", addr)
	if err != nil {
		log.Panicln("Error listening:", err.Error())
	}
	defer listener.Close()

	log.Println("TCP Server: " + addr)

	server_tcp.Listen(listener)
}



func setupHttpServer(host string, port string){
	addrHttp := host + ":" + port

	router := server_http.NewRouter()

	log.Println("HTTP Server: " + addrHttp)
	log.Fatal(http.ListenAndServe(addrHttp, router))

}