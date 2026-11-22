//! A small S-expression reader for KiCad files.
//!
//! KiCad files are nested lists of atoms and quoted strings. Each list keeps
//! the byte range it came from, so a writer can remove an item from the
//! original text without re-printing the rest of the file.

use std::fmt;

#[derive(Clone, Debug, PartialEq)]
pub enum Node {
    /// An unquoted atom: a keyword or a number.
    Atom(String),
    /// A quoted string, unescaped.
    Str(String),
    /// A list, with the byte range `start..end` of its text (both parentheses included).
    List { items: Vec<Node>, start: usize, end: usize },
}

#[derive(Debug)]
pub struct ParseError {
    pub offset: usize,
    pub message: String,
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{} at byte {}", self.message, self.offset)
    }
}

impl std::error::Error for ParseError {}

/// Parse one top-level list (a whole KiCad file).
pub fn parse(text: &str) -> Result<Node, ParseError> {
    let bytes = text.as_bytes();
    let mut pos = 0;
    skip_ws(bytes, &mut pos);
    if pos >= bytes.len() || bytes[pos] != b'(' {
        return Err(ParseError {
            offset: pos,
            message: "expected '('".into(),
        });
    }
    let node = parse_list(bytes, &mut pos)?;
    Ok(node)
}

fn skip_ws(b: &[u8], pos: &mut usize) {
    while *pos < b.len() && (b[*pos] as char).is_ascii_whitespace() {
        *pos += 1;
    }
}

fn parse_list(b: &[u8], pos: &mut usize) -> Result<Node, ParseError> {
    // iterative, so deeply nested files cannot overflow the stack
    let mut stack: Vec<(Vec<Node>, usize)> = Vec::new();
    loop {
        skip_ws(b, pos);
        if *pos >= b.len() {
            return Err(ParseError {
                offset: *pos,
                message: "unexpected end of file".into(),
            });
        }
        match b[*pos] {
            b'(' => {
                stack.push((Vec::new(), *pos));
                *pos += 1;
            }
            b')' => {
                *pos += 1;
                let (items, start) = match stack.pop() {
                    Some(v) => v,
                    None => {
                        return Err(ParseError {
                            offset: *pos - 1,
                            message: "unbalanced ')'".into(),
                        })
                    }
                };
                let node = Node::List {
                    items,
                    start,
                    end: *pos,
                };
                match stack.last_mut() {
                    Some((parent, _)) => parent.push(node),
                    None => return Ok(node),
                }
            }
            b'"' => {
                let s = parse_string(b, pos)?;
                match stack.last_mut() {
                    Some((parent, _)) => parent.push(Node::Str(s)),
                    None => {
                        return Err(ParseError {
                            offset: *pos,
                            message: "string outside a list".into(),
                        })
                    }
                }
            }
            _ => {
                let start = *pos;
                while *pos < b.len() {
                    let c = b[*pos];
                    if c == b'(' || c == b')' || c == b'"' || (c as char).is_ascii_whitespace() {
                        break;
                    }
                    *pos += 1;
                }
                let atom = String::from_utf8_lossy(&b[start..*pos]).into_owned();
                match stack.last_mut() {
                    Some((parent, _)) => parent.push(Node::Atom(atom)),
                    None => {
                        return Err(ParseError {
                            offset: start,
                            message: "atom outside a list".into(),
                        })
                    }
                }
            }
        }
    }
}

fn parse_string(b: &[u8], pos: &mut usize) -> Result<String, ParseError> {
    let start = *pos;
    *pos += 1;
    let mut out: Vec<u8> = Vec::new();
    while *pos < b.len() {
        let c = b[*pos];
        if c == b'\\' && *pos + 1 < b.len() {
            let n = b[*pos + 1];
            out.push(match n {
                b'n' => b'\n',
                b't' => b'\t',
                b'r' => b'\r',
                other => other,
            });
            *pos += 2;
            continue;
        }
        if c == b'"' {
            *pos += 1;
            return Ok(String::from_utf8_lossy(&out).into_owned());
        }
        out.push(c);
        *pos += 1;
    }
    Err(ParseError {
        offset: start,
        message: "unterminated string".into(),
    })
}

/// Quote a string the way KiCad writes it.
pub fn quote(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            other => out.push(other),
        }
    }
    out.push('"');
    out
}

impl Node {
    /// The list's first atom (its keyword), if it is a list.
    pub fn head(&self) -> Option<&str> {
        match self {
            Node::List { items, .. } => match items.first() {
                Some(Node::Atom(a)) => Some(a.as_str()),
                _ => None,
            },
            _ => None,
        }
    }

    pub fn items(&self) -> &[Node] {
        match self {
            Node::List { items, .. } => items,
            _ => &[],
        }
    }

    pub fn span(&self) -> Option<(usize, usize)> {
        match self {
            Node::List { start, end, .. } => Some((*start, *end)),
            _ => None,
        }
    }

    /// The text of an atom or string.
    pub fn text(&self) -> Option<&str> {
        match self {
            Node::Atom(s) | Node::Str(s) => Some(s.as_str()),
            _ => None,
        }
    }

    /// Child lists whose keyword is `name`.
    pub fn all<'a>(&'a self, name: &'a str) -> impl Iterator<Item = &'a Node> + 'a {
        self.items().iter().filter(move |n| n.head() == Some(name))
    }

    /// The first child list whose keyword is `name`.
    pub fn get(&self, name: &str) -> Option<&Node> {
        self.items().iter().find(|n| n.head() == Some(name))
    }

    /// The k-th value after the keyword (0-based) as text.
    pub fn arg(&self, k: usize) -> Option<&str> {
        self.items().get(k + 1).and_then(|n| n.text())
    }

    pub fn num(&self, k: usize) -> Option<f64> {
        self.arg(k).and_then(|s| s.parse::<f64>().ok())
    }

    /// The first value of child `name`: `(width 0.25)` -> 0.25.
    pub fn child_num(&self, name: &str) -> Option<f64> {
        self.get(name).and_then(|n| n.num(0))
    }

    pub fn child_text(&self, name: &str) -> Option<&str> {
        self.get(name).and_then(|n| n.arg(0))
    }

    /// Every value after the keyword, as text.
    pub fn args(&self) -> Vec<&str> {
        self.items().iter().skip(1).filter_map(|n| n.text()).collect()
    }

    /// True when a bare flag such as `locked` is among the values, or a child
    /// `(locked yes)` says so.
    pub fn flag(&self, name: &str) -> bool {
        if self
            .items()
            .iter()
            .skip(1)
            .any(|n| matches!(n, Node::Atom(a) if a == name))
        {
            return true;
        }
        match self.get(name) {
            Some(n) => !matches!(n.arg(0), Some("no")),
            None => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_nested_lists_strings_and_spans() {
        let text = "(kicad_pcb (version 20260206) (net \"/A \\\"b\\\"\") (segment (start 1 2)))";
        let n = parse(text).unwrap();
        assert_eq!(n.head(), Some("kicad_pcb"));
        assert_eq!(n.get("version").unwrap().num(0), Some(20260206.0));
        assert_eq!(n.get("net").unwrap().arg(0), Some("/A \"b\""));
        let seg = n.get("segment").unwrap();
        let (s, e) = seg.span().unwrap();
        assert_eq!(&text[s..e], "(segment (start 1 2))");
        assert_eq!(seg.get("start").unwrap().num(1), Some(2.0));
    }

    #[test]
    fn quote_round_trips() {
        let q = quote("a\"b\\c");
        let n = parse(&format!("(x {})", q)).unwrap();
        assert_eq!(n.arg(0), Some("a\"b\\c"));
    }

    #[test]
    fn flags() {
        let n = parse("(via locked (at 0 0))").unwrap();
        assert!(n.flag("locked"));
        let n = parse("(segment (locked yes))").unwrap();
        assert!(n.flag("locked"));
        let n = parse("(segment (locked no))").unwrap();
        assert!(!n.flag("locked"));
    }
}
