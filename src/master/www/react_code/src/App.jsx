import { useState } from "react";
import { Navbar } from "./components/Navbar";
import { navbars } from "./static/Vars";
import { Field } from "./components/Field";
import { Ball } from "./components/Ball";
import { Line } from "./components/Line";

export default function App() {
  const [pageState, setPageState] = useState("Field");
  const handleNavbarClick = (newPageState) => {
    setPageState(newPageState);
  }
  return (
    <>
      <Navbar navbar={navbars} onNavbarClick={handleNavbarClick} />
      {pageState === "Field" && (
        <Field />
      )}

      {pageState === "Ball" && (
        <div>
          <Ball />
        </div>
      )}

      {pageState === "Line" && (
        <div>
          <Line />
        </div>
      )}
    </>
  )
}