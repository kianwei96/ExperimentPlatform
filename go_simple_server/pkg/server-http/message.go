package server_http

type MessagePush struct {
	Timestamp	string      `json:"timestamp"`
	Devices		[]Device `json:"devices"`
}

type Devices struct {
	Devices []Device `json:"devices"`
}


type Device struct {
	DeviceId	string	`json:"deviceId"`
	Result		int8	`json:"result"`
}

