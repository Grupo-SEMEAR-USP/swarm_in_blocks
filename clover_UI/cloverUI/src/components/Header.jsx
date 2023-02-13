import './styles/Header.css'



function Header() {
    return (
      <div id="Header">
          <div id='h_purple_gradient'></div>
            <div id='he_1'>
              <img src='/assets/img/logo_header.png'/>
            </div>
            
            <div id='he_2'>
              <a href='http://localhost/' id='he_links'>Home</a>
              <a id='he_links'>About us</a>
              <a href='http://localhost/ground_station/ground_station.html' target="_blank;" id='he_links'>Kit tools</a>
              <a href="http://localhost/swarm_IDE/swarmIDE/www/index.html" target="_blank;" id='he_links'>Swarm in blocks</a>
            </div>
      </div>
    )
}

export default Header