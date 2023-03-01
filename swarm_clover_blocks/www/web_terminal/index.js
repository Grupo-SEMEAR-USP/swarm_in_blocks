// Page js

var terminalCounter = 0;
document.getElementById("open-terminal-button").addEventListener("click", function() {
    console.log("Creating new terminal")
    // setTimeout(function() {
    //     console.log("Esta mensagem será exibida após 1 segundo.");
    //   }, 1000);
    terminalCounter++;
    var terminalDiv = document.createElement("div");
    terminalDiv.className = webTerminal;
    terminalDiv.id = terminalCounter - 1;
    document.body.appendChild(terminalDiv);
    // var webTerminal = new webTerminal(document.getElementById("terminal" + terminalCounter));
    console.log("Terminal div: ", terminalDiv);
    let terminal = new webTerminal(terminalDiv.id);
});