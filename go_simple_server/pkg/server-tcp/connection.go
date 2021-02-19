package server_tcp

import "net"

type tcpConn struct {
	Name string
	Conn net.Conn
}

var currConn = &tcpConn{
	Name: "test string",
}

func GetTcpConn() net.Conn{
	return currConn.Conn
}

func GetTcpConnTest() string{
	return currConn.Name
}