import { toast, ToastContainer } from 'react-toastify';
import "react-toastify/dist/ReactToastify.css";

// import "./Notifications.css";
import "./Notifications.css";

const notifyManager = {
  "info": (msg) => { return toast.info(msg); },
  "success": (msg) => { return toast.success(msg); },
  "warn": (msg) => { return toast.warn(msg); },
  "error": (msg) => {
    return toast(msg, {
      type: 'error',
      autoClose: false,
    });
  },
};

export const notificationMsg = (level, message) => {
  try {
    notifyManager[level](message);
  } catch (e) {
    console.error(e);
  }
}

export const NotificationContainer = () => {
  return <ToastContainer
    position="bottom-right"
    closeOnClick={true}
    autoClose={3000}
    theme="dark"
  />
}
