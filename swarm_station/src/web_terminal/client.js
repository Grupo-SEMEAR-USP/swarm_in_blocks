// import FitAddon from '/node_modules/xterm-addon-fit/lib/xterm-addon-fit.js';
// Create a conection between the frontend and backend 

// let divs = JSON.parse(localStorage.getItem('ids')) || [];

class webTerminal {

    constructor(terminalDiv_id) {

        // TerminalDiv_id ->1,2,3...
        // divs.push(terminalDiv_id);
        // localStorage.setItem('ids', JSON.stringify(divs));
        // let ids = JSON.parse(localStorage.getItem('ids'));
        // console.log("Divs: ", ids);
        
        let server = "ws://localhost:606" + terminalDiv_id

        // this.socket = new WebSocket("ws://localhost:6060");
        this.socket = new WebSocket(server);
        console.log("This socket: ", this.socket);

        // Library used to control terminal's size
        this.fitAddon = new FitAddon.FitAddon();

        // Instantiate a xterm object
        this.terminal = new window.Terminal({
          cursorBlink: true,
          theme: { foreground: "#44ff9b", background: '#000000'},
          allowTransparency: true,
          scrollback: 1000,
        });
    

        this.terminal.loadAddon(this.fitAddon);
        
        // Open terminal element from html file
        if (this.terminal != null) {
          this.terminal.open(document.getElementById(terminalDiv_id));
          // Fit terminal element in the definied container
          this.fitAddon.fit();
        } else {
          console.log("Failed to open terminal element");
        }

        this.openTerminalButton = document.getElementById("open-terminal");

        // Setting control variables
        this.getUser = true;
        this.userData;
        this.firstMsg = true;
        this.firstConfigs = true;
        this.welcomeMsg;
        this.currentDirectory = [];
        this.auxCurDir=false;
        this.aux1 = 0;
        this.newLine = false;
        this.numberLine = 0;
        this.terminalDiv_id = terminalDiv_id;

        this.update()
    }
  
    init() {

        if (this.terminal._initialized) return;
    
        this.terminal._initialized = true;
    
        this.terminal.prompt = () => {
            this.terminal.write(`\r\n${this.userData.username}@${this.userData.hostname}:$ `);
        };

        this.prompt();

        let clientInput;
    
        // Listen to clients inputs on the
        this.terminal.onData(dataInput => {
            console.log("onData")

            if (this.firstConfigs === true) {
                for (let i = 0; i < this.welcomeMsg.length; i++) {
                    this.terminal.write('\b \b');
                }
                this.socket.send('\n');
            }else{
                this.socket.send(dataInput);
            }

            // // Check if the client reached a new line
            // if(this.terminal._core.buffer.x === 1){
            //     this.newLine = true;
            //     this.numberLine += 1;
            // }        
      
        });
    }
  
    prompt() {
        // First time running it will print the welcome message in the terminal
        if(this.firstMsg === true){
            this.terminal.write('\r\n╔═══╗╔╗╔╗╔╗╔═══╗╔═══╗╔═╗╔═╗    ╔══╗╔═╗ ╔╗    ╔══╗ ╔╗   ╔═══╗╔═══╗╔╗╔═╗╔═══╗');
            this.terminal.write('\r\n║╔═╗║║║║║║║║╔═╗║║╔═╗║║║╚╝║║    ╚╣╠╝║║╚╗║║    ║╔╗║ ║║   ║╔═╗║║╔═╗║║║║╔╝║╔═╗║');
            this.terminal.write('\r\n║╚══╗║║║║║║║║ ║║║╚═╝║║╔╗╔╗║     ║║ ║╔╗╚╝║    ║╚╝╚╗║║   ║║ ║║║║ ╚╝║╚╝╝ ║╚══╗');
            this.terminal.write('\r\n╚══╗║║╚╝╚╝║║╚═╝║║╔╗╔╝║║║║║║     ║║ ║║╚╗║║    ║╔═╗║║║ ╔╗║║ ║║║║ ╔╗║╔╗║ ╚══╗║');
            this.terminal.write('\r\n║╚═╝║╚╗╔╗╔╝║╔═╗║║║║╚╗║║║║║║    ╔╣╠╗║║ ║║║    ║╚═╝║║╚═╝║║╚═╝║║╚═╝║║║║╚╗║╚═╝║');
            this.terminal.write('\r\n╚═══╝ ╚╝╚╝ ╚╝ ╚╝╚╝╚═╝╚╝╚╝╚╝    ╚══╝╚╝ ╚═╝    ╚═══╝╚═══╝╚═══╝╚═══╝╚╝╚═╝╚═══╝');
            this.terminal.write('\r\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');
            this.terminal.write('\r\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\r\n');
            this.firstMsg = false;
            this.welcomeMsg = 'Welcome to the web terminal application, press enter to start using ';
            this.terminal.write(``) ;
            this.terminal.write(`\r\n${this.welcomeMsg}`);    
                                            
        }else{
            this.terminal.write(`\r\n${this.userData.username}@${this.userData.hostname}:$ `);
        }
    }

    update(){
        this.socket.onmessage = (event) => {
            let aux2;
            if(this.getUser===true){
                // Receive user data
                this.userData = JSON.parse(event.data);
                this.getUser = false;
            }else if(this.firstConfigs===true){
                    //dont write anything to the client
                    this.firstConfigs = false;
                    return;
            }else if(this.auxCurDir===true){
                    var currentDirectoryBack = event.data;
                    if (currentDirectoryBack[0]==='/'){
                        this.currentDirectory = currentDirectoryBack.split("\n")[0];
                    }
                    this.aux1 += 1;
                    if(this.aux1 === 2){
                        this.auxCurDir = false;
                        this.aux1 = 0;
                        aux2 = true;
                    }
            }else {
                // console.log(this.terminal.cols);
                // console.log(this.terminal._core.buffer.x);
                this.terminal.write(event.data);
            }
        }
        this.checkButton(this.socket);
        this.init();
    }

    getCurrentDir(){
        // Get current directory
        this.auxCurDir = true;
        let pwdclientInput = String.fromCharCode(112) + String.fromCharCode(119) + String.fromCharCode(100);
        this.socket.send(pwdclientInput + '\n');
        // console.log(`CurrentDir: ${currentDirectory}`);
        let curDirLenght = this.currentDirectory.length - this.userData.username.length - 6;
        // console.log(`CurrentDirLenght: ${curDirLenght}`);
        return curDirLenght
    }

    checkButton(socket){

        // const msg = "New Terminal";
        // const asciiMsg = [];

        // for (let i = 0; i < msg.length; i++) {
        //     asciiMsg.push(msg.charCodeAt(i));
        // }
        // console.log(asciiMsg);
        let counter = 1;

        this.openTerminalButton.addEventListener("click", function() {
            if(counter<10){
                counter +=1;

                const msg = "New Terminal" + counter;
                const asciiMsg = [];
    
                for (let i = 0; i < msg.length; i++) {
                    asciiMsg.push(msg.charCodeAt(i));
                }
                // console.log("Botão apertado");
                socket.send(asciiMsg);   
            }else{
                console.log("maximum number of terminals exceeded");
            }
 
        });

    }

}

// let terminal = new webTerminal();


