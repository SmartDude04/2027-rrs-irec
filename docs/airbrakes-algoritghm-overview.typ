#import "@preview/cetz:0.5.3"

#set page(
  numbering: "1",
  paper: "us-letter"
)
#set text(
  font: "New Computer Modern",
  size: 12pt
)
#set par(
  justify: true,
)

#let state(title, description, pos, color, border) = {
  import cetz.draw: *

  let (x, y) = pos

  rect(
    (x, y),
    (x + 5, y + 3),
    fill: color,
    stroke: border,
  )

  content(
    (x + 2.5, y + 1),
    [
      #align(center)[
        #strong(title) \
        #text(size: 9pt)[#description]
      ]
    ],
  )
}

#let arrow(start, end, content_text, arrow) = {
  import cetz.draw: *
  if arrow {
    line(start, end, mark: (end: ">"))
  } else {
    line(start, end)
  }

  content(((start.at(0) + end.at(0)) / 2, (start.at(1) + end.at(1)) / 2 - 0.35), content_text)
}

#cetz.canvas({
  import cetz.draw: *

  
})