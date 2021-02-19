package server_http

import (
	"fmt"
	"go_simple_server/pkg/server-tcp"
	"log"
	"net/http"
)
func Index(w http.ResponseWriter, r *http.Request) {
	fmt.Fprintf(w, "Main Index Page!\n")
	fmt.Println(server_tcp.GetTcpConnTest())
}

func UpdateImageHandler(w http.ResponseWriter, r *http.Request){
	ids, ok := r.URL.Query()["imageId"]
	if !ok || len(ids[0]) < 1 {
		log.Println("Url Param 'imageId' is missing")
		return
	}
	id := ids[0]
	log.Println("Url Param 'imageId' is: " + id)

	err := server_tcp.SendMsg(server_tcp.GetTcpConn(),id)
	if err != nil {
		// TODO: error handler msg back to the sender
		log.Println(err)
		w.WriteHeader(500)
	} else {
		w.WriteHeader(200)
	}
}