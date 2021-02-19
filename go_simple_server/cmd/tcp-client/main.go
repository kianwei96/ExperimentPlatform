package main
import (
	"net"
	"os"
	"time"
)

func main() {
	strEcho := "Halo"
	servAddr := "localhost:8080"
	tcpAddr, err := net.ResolveTCPAddr("tcp", servAddr)
	if err != nil {
		println("ResolveTCPAddr failed:", err.Error())
		os.Exit(1)
	}

	conn, err := net.DialTCP("tcp", nil, tcpAddr)
	if err != nil {
		println("Dial failed:", err.Error())
		os.Exit(1)
	}

	for i:=0; i<5; i++{
		_, err = conn.Write([]byte(strEcho))
		if err != nil {
			println("Write to server failed:", err.Error())
			os.Exit(1)
		}

		println("write to server = ", strEcho)

		reply := make([]byte, 1024)

		res, err := conn.Read(reply)
		if err != nil {
			println("Write to server failed:", err.Error())
			os.Exit(1)
		} else {
			println("respond: ", res)
		}

		println("reply from server=", string(reply))

		time.Sleep(1 * time.Second)
	}


	conn.Close()
}