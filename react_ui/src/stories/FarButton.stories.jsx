import FarButton from "../components/FarButton";

export default {
 title: "Components/SwarmUI/FarButton",
 component: FarButton,
 argTypes: {
   handleClick: { action: "logging it in action section" },
 }
};

const Template = (args) => <FarButton {...args} />;

export const Green = Template.bind({});

Green.args = {
 backgroundColor: "green",
 label: "Middle Green button",
 size: "md",
 color: "black",
};


export const Pink = Template.bind({});

Pink.args = {
 backgroundColor: "pink",
 label: "Middle Pink Button",
 color: "blue",
 size: "md",
};


export const Small = Template.bind({});

Small.args = {
 backgroundColor: "blue",
 label: "Small button",
 size: "sm",
 color: "white",
};

export const Large = Template.bind({});

Large.args = {
 label: "Very very very large button",
 size: "lg",
 color: "black",
 backgroundColor: "red",
};