// Page js

var terminalCounter = 0;
document.getElementById("open-terminal-button").addEventListener("click", function() {
    console.log("Creating new terminal")
    terminalCounter++;
    var terminalDiv = document.createElement("div");
    terminalDiv.className = webTerminal;
    terminalDiv.id = terminalCounter - 1;
    document.body.appendChild(terminalDiv);
    terminalDiv.style.marginBottom = "5px"; // adiciona um estilo CSS para ajustar a margem inferior
    // document.body.appendChild(document.createElement("br"));
    // var webTerminal = new webTerminal(document.getElementById("terminal" + terminalCounter));
    console.log("Terminal div: ", terminalDiv);
    let terminal = new webTerminal(terminalDiv.id);
});
