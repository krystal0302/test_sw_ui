import "./ArtifactCarousel.css";

import { useState, useContext } from "react";
import Carousel from "react-multi-carousel";

import SearchBox from './SearchBox';
import ArtifactCard from "./ArtifactCard";
import { TokenContext } from '../../utils/TokenContextProvider';
import { useGetArtifacts } from "../../hooks/FetchHooks";


const responsive = {
  desktop: {
    breakpoint: { max: 3000, min: 1024 },
    items: 4
  },
  tablet: {
    breakpoint: { max: 1024, min: 464 },
    items: 2
  },
  mobile: {
    breakpoint: { max: 464, min: 0 },
    items: 1
  }
};

const ArtifactCarousel = () => {
  const [additionalTransform, setAdditionalTransform] = useState(0);
  const [keyword, setKeyword] = useState('');
  const [artifacts, setArtifacts] = useState([]);

  const { token } = useContext(TokenContext);

  const handles = {
    onSuccess: (data) => { setArtifacts(data.artifacts); },
    onError: (err) => { console.error(err) },
    refetchInterval: 1000
  }
  const dArtifacts = useGetArtifacts(token, handles);

  const searchArtifact = (key) => { setKeyword(key); };

  return (
    <>
      <SearchBox item="artifact" callbacks={{ search: searchArtifact }} />
      <Carousel
        ssr={false}
        partialVisbile={false}
        itemClass="slider-image-item"
        responsive={responsive}
        containerClass="carousel-container-with-scrollbar"
        additionalTransfrom={-additionalTransform}
        beforeChange={nextSlide => {
          if (nextSlide !== 0 && additionalTransform !== 150) {
            setAdditionalTransform(150);
          }
          if (nextSlide === 0 && additionalTransform === 150) {
            setAdditionalTransform(0);
          }
        }}
      >
        {artifacts
          .filter(a => a.artifact_name.includes(keyword))
          .map((artifact, index) => <ArtifactCard key={index} content={artifact} token={token} />)}
      </Carousel>
    </>
  );
}

export default ArtifactCarousel;
