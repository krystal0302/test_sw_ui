import "semantic-ui-css/semantic.min.css";
import "react-multi-carousel/lib/styles.css";
import "../css/sdk_styles.css"

import React, { useState } from "react";
import UAParser from "ua-parser-js";
import LoadingOverlay from 'react-loading-overlay';
/** 
fix issue: 
  LoadingOverlayWrapper2: prop type `styles.content` is invalid; 
  it must be a function, usually from the `prop-types` package, but received `undefined`.
*/
LoadingOverlay.propTypes = undefined


import ArtifactCarousel from "../components/Dashboard/ArtifactCarousel";
import Section from "../components/Section";
import WrapperList from "../components/Dashboard/WrapperList";


const SdkDashboard = () => {
  const [isActive, setActive] = useState(false);

  function handleBlockOverlay() {
    setActive(true);
    setTimeout(() => { setActive(false); }, 3000);
  };

  return (
    <LoadingOverlay
      active={isActive}
      spinner
      text='Depoly the wrapper...'
    >
      {/* EXTEND ver. */}
      {/* <div className="content-wrapper farobot-dark-mode" style={{ minHeight: '900px' }}> */}
      {/* COLLAPSE ver. */}
      <div className="farobot-dark-mode" style={{ minHeight: "calc(100vh - 3.5rem)" }}>
        <div className="content">
          <div>
            <Section>
              <div style={{ textAlign: 'center', fontSize: '20px', padding: '1rem' }}>Wrapper List</div>
              <WrapperList farOverlay={handleBlockOverlay} />
            </Section>
            <Section>
              <div style={{ textAlign: 'center', fontSize: '20px', padding: '1rem' }}>Artifact List</div>
              <ArtifactCarousel />
            </Section>
          </div>
        </div>
      </div>
    </LoadingOverlay>
  );
};

SdkDashboard.getInitialProps = ({ req }) => {
  let userAgent = (req) ? req.headers["user-agent"] : navigator.userAgent;
  const parser = new UAParser();
  parser.setUA(userAgent);
  const result = parser.getResult();
  const deviceType = (result.device && result.device.type) || "desktop";
  return { deviceType };
};

export default SdkDashboard;