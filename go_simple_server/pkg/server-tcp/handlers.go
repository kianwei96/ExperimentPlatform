package server_tcp

import (
	"errors"
	"fmt"
	"log"
	"net"
	"strconv"
)

func Listen(listener net.Listener) {
	for {
		conn, err := listener.Accept()
		currConn.Conn = conn
		if err != nil {
			log.Panicln("Error accepting: ", err.Error())
		}
		// Handle connections in a new goroutine.
		go handleRequest(conn)
	}
}

func handleRequest(conn net.Conn) {
	fmt.Println("handle request")
	buf := make([]byte, 1024)
	i := 0
	defer conn.Close()

	for true {
		_, err := conn.Read(buf)
		if err != nil {
			fmt.Println("Error reading:", err.Error())
			if conn == currConn.Conn{
				currConn.Conn = nil
			}
			return
		} else {
			fmt.Println("TCP Message received: " + string(buf))
			msg := strconv.Itoa(i)
			i = (i+1) % 6
			conn.Write([]byte(msg))
		}
	}
}

func SendMsg(conn net.Conn, msg string) (err error){
	if conn  == nil {
		err = errors.New("No TCP conn")
	} else {
		_, err = conn.Write([]byte(msg))
	}

	return err
}