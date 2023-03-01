import './styles/Header.css'



function Header() {
    return (
      <div id="Header">
          <div id='h_purple_gradient'></div>
            <div id='he_1'>
              <img src='/assets/img/logo_header.png'/>
            </div>
            
            <div id='he_2'>
              <a href='/' id='he_links'>Home</a>
              <a href='http://www.semear.eesc.usp.br/equipe-atena/' target="_blank;" id='he_links'>About us</a>
              <a href='swarm_station/ground_station.html' target="_blank;" id='he_links'>Swarm station</a>
              <a href="swarm_clover_blocks/www/index.html" target="_blank;" id='he_links'>Swarm in blocks</a>
            </div>
      </div>
    )
}

export default Header