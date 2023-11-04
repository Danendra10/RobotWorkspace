import PropTypes from "prop-types";
import React from "react";
import { icon_iris_black } from "../assets";

export const Navbar = (props) => {
  const handleClick = (name) => {
    props.onNavbarClick(name);
  };
  return (
    <nav className="relative px-4 py-4 flex justify-between items-center bg-white">
      <div className="text-3xl font-bold leading-none">
        <img src={icon_iris_black} alt="" />
      </div>
      <ul className="absolute top-1/2 left-1/2 transform -translate-y-1/2 -translate-x-1/2 lg:flex lg:mx-auto lg:flex lg:items-center lg:w-auto lg:space-x-6">
        <li className="text-gray-300">
          <svg
            xmlns="http://www.w3.org/2000/svg"
            fill="none"
            stroke="currentColor"
            className="w-4 h-4 current-fill"
            viewBox="0 0 24 24"
          >
            <path
              strokeLinecap="round"
              strokeLinejoin="round"
              strokeWidth="2"
              d="M12 5v0m0 7v0m0 7v0m0-13a1 1 0 110-2 1 1 0 010 2zm0 7a1 1 0 110-2 1 1 0 010 2zm0 7a1 1 0 110-2 1 1 0 010 2z"
            />
          </svg>
        </li>
        {props.navbar.map((item, index) => (
          <React.Fragment key={index}>
            <li
              className="hover:cursor-pointer"
              onClick={() => handleClick(item.name)} // Call the handleClick function on click
            >
              <a className="text-sm text-gray-400 hover:text-gray-500">
                {item.name}
              </a>
            </li>
            <li className="text-gray-300">
              <svg
                xmlns="http://www.w3.org/2000/svg"
                fill="none"
                stroke="currentColor"
                className="w-4 h-4 current-fill"
                viewBox="0 0 24 24"
              >
                <path
                  strokeLinecap="round"
                  strokeLinejoin="round"
                  strokeWidth="2"
                  d="M12 5v0m0 7v0m0 7v0m0-13a1 1 0 110-2 1 1 0 010 2zm0 7a1 1 0 110-2 1 1 0 010 2zm0 7a1 1 0 110-2 1 1 0 010 2z"
                />
              </svg>
            </li>
          </React.Fragment>
        ))}
      </ul>
    </nav>
  );
};

Navbar.propTypes = {
  navbar: PropTypes.arrayOf(
    PropTypes.shape({
      name: PropTypes.string.isRequired,
    }),
  ).isRequired,
  onNavbarClick: PropTypes.func.isRequired, // Define onNavbarClick as a required function prop
};
