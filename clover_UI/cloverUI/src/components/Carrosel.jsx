import Slider from "react-slick";
import "slick-carousel/slick/slick.css"; 
import "slick-carousel/slick/slick-theme.css";

import './styles/Carrosel.css'
import Cards from './Cards';

const settings = {
    dots: false,
    infinite: true,
    speed: 500,
    slidesToShow: 3,
    slidesToScroll: 1,
    initialSlide: 0,
    responsive: [
      {
        breakpoint: 1024,
        settings: {
          slidesToShow: 3,
          slidesToScroll: 3,
          infinite: true,
          dots: true
        }
      }
    ]
};

function Carrosel() {
    return (
      <div id="Carrosel">
        <Slider
            {...settings}
        >
            <Cards
                title="Plug and Play"
                subtitle="Clover offers a plug and play platform with integrated software and hardware solution for beginners in drones and robotics"
                img_src="src/assets/PlugNPlay.svg"
            />
            <Cards
                title="Open Source"
                subtitle="The development of all Clovers features is available online and can be improved by the community, i.e. it is an open source project"
                img_src="src/assets/OpenSource.svg"
            />
            <Cards
                title="Clover Swarm"
                subtitle="With Swarm in Blocks platform you can build swarms of Clovers in an easy and integrated way"
                img_src="src/assets/CloverSwarm.svg"
            />
            <Cards
                title="Educational"
                subtitle="Due to the abstractions of the Clover platform, explored further in Swarm in blocks,  a child can explore the wonders of a drone swarm"
                img_src="src/assets/Educational.svg"
            />
            <Cards
                title="Aplications"
                subtitle="There are several applications for drones today (agriculture, industry, etc.) and even more so when we talk about swarms of drones"
                img_src="src/assets/Applications.svg"
            />
        </Slider>
      </div>
    )
}

export default Carrosel