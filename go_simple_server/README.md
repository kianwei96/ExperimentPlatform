cmd/tcp-server is the one you need to export/install. cmd/tcp-client is for testing without using the phone app.

If you want to use >5 images or have different behaviour, you can modify the handlers.go file in pkg/server-tcp. The handlers for http server only check for id parameter field before passing the data.

Use "go build" command to create executable.
