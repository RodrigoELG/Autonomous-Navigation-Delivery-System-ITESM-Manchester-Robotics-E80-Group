//author AECL

package main

import (
    "context"
    "flag"
    "fmt"
    "net/http"
    "github.com/golang/glog" // Asegúrate de tener esta librería instalada

    "google.golang.org/grpc"
	insecure "google.golang.org/grpc/credentials/insecure" 

    "github.com/grpc-ecosystem/grpc-gateway/v2/runtime"

    // Esta importación es CRÍTICA y DEBE coincidir con el go_package en tu .proto
    // Y con la ubicación de los archivos Go generados.
    // Asumiendo que el main.go está en interfaz-puzzlebot/ y los protos generados en generated_proto_go/
    gw "interfaz-puzzlebot/generated_proto_go" // Esta es la ruta para Go
)

var (
    // Asegúrate de que esta sea la IP y puerto de tu servidor gRPC (ros2-grpc-wrapper.py)
    // en la Jetson. Si el ros2-grpc-wrapper.py corre en la misma máquina que el gateway, 127.0.0.1 está bien.
    // Si el ros2-grpc-wrapper.py corre en la Jetson, DEBES poner la IP de la Jetson aquí.
    grpcServerEndpoint = flag.String("grpc-server-endpoint", "127.0.0.1:7042", "gRPC svr endpoint")
    gw_port = "8042" // Puerto donde el gRPC-Gateway escuchará peticiones HTTP
)

func run() error {
    ctx := context.Background()
    ctx, cancel := context.WithCancel(ctx)
    defer cancel()

    mux := runtime.NewServeMux()
    // Opciones de conexión gRPC. Con WithInsecure() para pruebas.
    opts := []grpc.DialOption{grpc.WithTransportCredentials(insecure.NewCredentials())} // Corregido para gRPC v1.23+
    // Para versiones más antiguas de gRPC (como la que podrías tener con grpc-go v1.15.0), usa:
    // opts := []grpc.DialOption{grpc.WithInsecure()}

    // Registrar el handler del servicio RPCDemo desde el endpoint del servidor gRPC
    err := gw.RegisterRPCDemoHandlerFromEndpoint(ctx, mux, *grpcServerEndpoint, opts)
    if err != nil {
        return fmt.Errorf("failed to register gateway: %w", err)
    }

    fmt.Println("Starting Gateway HTTP server on port " + gw_port)
    // Iniciar el servidor HTTP del gateway
    return http.ListenAndServe(":"+gw_port, mux)
}

func main() {
    flag.Parse()
    defer glog.Flush() // Asegura que los logs se escriban antes de salir
    fmt.Println("Starting Gateway...")
    if err := run(); err != nil {
        glog.Fatal(err) // Muestra el error y termina el programa
    }
}