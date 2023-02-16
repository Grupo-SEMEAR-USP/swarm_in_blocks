const WebSocket = require('ws');
var os = require('os');
var pty = require('node-pty');

let portCounter = 6060;

class Server{
    constructor() {
        this.port = portCounter++;
        this.username = os.userInfo().username;
        this.hostname = os.hostname();
        this.serverCount = 1;
        this.createServer();


    }

    createServer(){

        this.wss = new WebSocket.Server({ port: this.port });
        console.log(`Web Terminal server is running at port ${this.port} `);
        this.startComunication()
        
    }

    startComunication(){

        var ptyProcess = pty.spawn(os.platform() === 'win32' ? 'powershell.exe' : 'bash', [], {
            name: 'xterm-color',
            cols: 81,
            cwd: process.env.HOME,
            env: process.env,
        })    
        this.wss.on('connection', (ws) => {
            console.log("New session inicialized");

            ws.send(JSON.stringify({username: this.username, hostname: this.hostname}));
        
            ws.on('message', (clientInput) => {

                const clientInputString = clientInput.toString();

                if(clientInputString === '78,101,119,32,84,101,114,109,105,110,97,108'){
                    if(this.port === 6060){
                        console.log("Creating new websocket connection");
                        console.log('server'+ this.serverCount);
                        // let serverName = 'server' + this.serverCount;
                        global['server' + this.serverCount] = new Server();
                        this.serverCount++;
                    }

                }else{
                    ptyProcess.write(clientInput);
                }
                
                // if(convMsg === "Create new terminal"){
                //     console.log("Criar novo server");
                // }else{
                //     ptyProcess.write(clientInput);
                // }

            });
        
            ptyProcess.on('data', (data) => {
                ws.send(data);
            });
        });
    }
}

firstServer = new Server();
secondServer = new Server();
