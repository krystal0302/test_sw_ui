import ConfigParamRow from "../components/ConfigParamRowSample";

export default {
 title: "Components/SwarmUI/ConfigParamRow",
 component: ConfigParamRow,
 argTypes: {
   handleClick: { action: "logging it in action section" },
 }
};

const Template = (args) => <ConfigParamRow {...args} />;

export const Style1 = Template.bind({});

Style1.args = {
};


export const Style2 = Template.bind({});

Style2.args = {
};


