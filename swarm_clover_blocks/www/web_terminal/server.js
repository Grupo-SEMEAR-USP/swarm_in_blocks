const WebSocket = require('ws');
var os = require('os');
var pty = require('node-pty');

let portCounter = 6060;

class Server{
    constructor() {
        this.port = portCounter++;
        this.username = os.userInfo().username;
        this.hostname = os.hostname();
        this.serverCount = 2;
        this.serversList = [];
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
            // Send client's username and hostname to frontend
            ws.send(JSON.stringify({username: this.username, hostname: this.hostname}));
        
            ws.on('message', (clientInput) => {

                let clientInputString = clientInput.toString();
                // last char indicates the id of the new terminal
                let clientInputArray = clientInputString.split(',');
                clientInputArray.pop();
                let lastChar = clientInputString.split(',').map(Number);
                lastChar = lastChar[lastChar.length - 1];
            
                clientInputString = clientInputArray.join(',');
                
                // If the server receives the correspoding message it will create a new server
                if(clientInputString === '78,101,119,32,84,101,114,109,105,110,97,108' && this.serverCount<10){
                    if(this.port === 6060){
                        if (!this.serversList.includes(lastChar)){
                            this.serversList.push(lastChar);
                            let serverName = "server " + this.serverCount;
                            console.log("Creating new websocket connection: " + serverName);
                            // console.log('server '+ this.serverCount);
                            global['server' + this.serverCount] = new Server();
                            this.serverCount++;
                        }else{
                            console.log("This server already exists")
                        }
                        // console.log("Lista", this.serversList)

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
