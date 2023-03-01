import './styles/Clover_tools.css'


function Clover_tools() {

    return (
    <div id="Clover_tools">
        <div class='container'>
            <div id='c_green_gradient'></div>
            <div id='sub_text'>
                <div id='sub_text_st'>
                    <div id='text_seg_st'>
                        Easy to 
                    </div>
                    <div id='eazy_text_st' >
                        Swarm
                    </div>
                </div>
                <p />
                Station
                <div class='text3'>
                Set of tools and resources integrated into a platform for handling a swarm of Clovers. The station provides the use of the main functionalities for handling swarms in an easy and integrated way. </div>
                <a href='/swarm_station/ground_station.html' target="_blank;">
                    <button class='button'> 
                        Get Started
                    </button>
                </a>
            </div>
            <div id='tools_cards'>
                <div id='c'>
                    <img src='/assets/img/3D_view.svg' id=''/>
                    <div id='text_card'>
                        <div id='text1_card'>3D view</div>
                        3d visualization of clover and surrounding environment.
                    </div>
                </div>

                <div id='c'>
                    <img src='/assets/img/Clover_documentation.svg' id=''/>
                    <div id='text_card'>
                        <div id='text1_card'>Real time information</div>
                        Acess real time information of the Clovers while they are online                    
                    </div>
                </div>

                <div id='c'>
                    <img src='/assets/img/Clover_blocks.svg' id=''/>
                    <div id='text_card'>
                        <div id='text1_card'>Clover blocks</div>
                        Code development through blocks to control a clover                   </div>
                </div>
            </div>
        </div>
    </div>
    )
}

export default Clover_tools