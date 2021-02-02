package main

import (
	"go_simple_server/pkg/server-main"
	"log"
	"os"
	"strconv"
)

const (
	HostDefault = "0.0.0.0"
	PortDefault = 8080
)

func main() {

	portTcp, err := strconv.Atoi(os.Args[1])
	if err != nil{
		log.Panicln(err)
	}
	portHttp, err := strconv.Atoi(os.Args[2])
	if err != nil{
		log.Panicln(err)
	}

	server_main.SetupServers(HostDefault, strconv.Itoa(portTcp), strconv.Itoa(portHttp))
	for true {}
}
