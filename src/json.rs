//! Just enough JSON to read a `.kicad_pro` project file.

use std::collections::BTreeMap;

#[derive(Clone, Debug, PartialEq)]
pub enum Json {
    Null,
    Bool(bool),
    Num(f64),
    Str(String),
    Arr(Vec<Json>),
    Obj(BTreeMap<String, Json>),
}

impl Json {
    pub fn get(&self, key: &str) -> Option<&Json> {
        match self {
            Json::Obj(m) => m.get(key),
            _ => None,
        }
    }

    /// Follow a path of keys: `j.path(&["net_settings", "classes"])`.
    pub fn path(&self, keys: &[&str]) -> Option<&Json> {
        let mut cur = self;
        for k in keys {
            cur = cur.get(k)?;
        }
        Some(cur)
    }

    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Json::Num(n) => Some(*n),
            _ => None,
        }
    }

    pub fn as_str(&self) -> Option<&str> {
        match self {
            Json::Str(s) => Some(s.as_str()),
            _ => None,
        }
    }

    pub fn as_arr(&self) -> &[Json] {
        match self {
            Json::Arr(a) => a,
            _ => &[],
        }
    }

    pub fn as_obj(&self) -> Option<&BTreeMap<String, Json>> {
        match self {
            Json::Obj(m) => Some(m),
            _ => None,
        }
    }
}

pub fn parse(text: &str) -> Result<Json, String> {
    let b = text.as_bytes();
    let mut p = 0;
    let v = value(b, &mut p)?;
    ws(b, &mut p);
    if p != b.len() {
        return Err(format!("trailing text at byte {}", p));
    }
    Ok(v)
}

fn ws(b: &[u8], p: &mut usize) {
    while *p < b.len() && matches!(b[*p], b' ' | b'\t' | b'\n' | b'\r') {
        *p += 1;
    }
}

fn value(b: &[u8], p: &mut usize) -> Result<Json, String> {
    ws(b, p);
    if *p >= b.len() {
        return Err("unexpected end of JSON".into());
    }
    match b[*p] {
        b'{' => {
            *p += 1;
            let mut m = BTreeMap::new();
            ws(b, p);
            if *p < b.len() && b[*p] == b'}' {
                *p += 1;
                return Ok(Json::Obj(m));
            }
            loop {
                ws(b, p);
                let k = match value(b, p)? {
                    Json::Str(s) => s,
                    _ => return Err(format!("object key is not a string at byte {}", p)),
                };
                ws(b, p);
                if *p >= b.len() || b[*p] != b':' {
                    return Err(format!("expected ':' at byte {}", p));
                }
                *p += 1;
                let v = value(b, p)?;
                m.insert(k, v);
                ws(b, p);
                match b.get(*p) {
                    Some(b',') => *p += 1,
                    Some(b'}') => {
                        *p += 1;
                        return Ok(Json::Obj(m));
                    }
                    _ => return Err(format!("expected ',' or '}}' at byte {}", p)),
                }
            }
        }
        b'[' => {
            *p += 1;
            let mut a = Vec::new();
            ws(b, p);
            if *p < b.len() && b[*p] == b']' {
                *p += 1;
                return Ok(Json::Arr(a));
            }
            loop {
                a.push(value(b, p)?);
                ws(b, p);
                match b.get(*p) {
                    Some(b',') => *p += 1,
                    Some(b']') => {
                        *p += 1;
                        return Ok(Json::Arr(a));
                    }
                    _ => return Err(format!("expected ',' or ']' at byte {}", p)),
                }
            }
        }
        b'"' => {
            *p += 1;
            let mut s: Vec<u8> = Vec::new();
            while *p < b.len() && b[*p] != b'"' {
                if b[*p] == b'\\' && *p + 1 < b.len() {
                    *p += 1;
                    match b[*p] {
                        b'n' => s.push(b'\n'),
                        b't' => s.push(b'\t'),
                        b'r' => s.push(b'\r'),
                        b'b' => s.push(8),
                        b'f' => s.push(12),
                        b'u' if *p + 4 < b.len() => {
                            let hex = std::str::from_utf8(&b[*p + 1..*p + 5]).unwrap_or("0");
                            let cp = u32::from_str_radix(hex, 16).unwrap_or(0xfffd);
                            let ch = char::from_u32(cp).unwrap_or('\u{fffd}');
                            let mut buf = [0u8; 4];
                            s.extend_from_slice(ch.encode_utf8(&mut buf).as_bytes());
                            *p += 4;
                        }
                        other => s.push(other),
                    }
                } else {
                    s.push(b[*p]);
                }
                *p += 1;
            }
            *p += 1;
            Ok(Json::Str(String::from_utf8_lossy(&s).into_owned()))
        }
        b't' if b[*p..].starts_with(b"true") => {
            *p += 4;
            Ok(Json::Bool(true))
        }
        b'f' if b[*p..].starts_with(b"false") => {
            *p += 5;
            Ok(Json::Bool(false))
        }
        b'n' if b[*p..].starts_with(b"null") => {
            *p += 4;
            Ok(Json::Null)
        }
        _ => {
            let start = *p;
            while *p < b.len() && matches!(b[*p], b'-' | b'+' | b'.' | b'e' | b'E' | b'0'..=b'9') {
                *p += 1;
            }
            let t = std::str::from_utf8(&b[start..*p]).unwrap_or("");
            t.parse::<f64>()
                .map(Json::Num)
                .map_err(|_| format!("bad value at byte {}", start))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_a_project_fragment() {
        let j = parse(
            r#"{"net_settings": {"classes": [{"name": "Default", "clearance": 0.2, "x": null, "y": true}],
                          "netclass_assignments": {"/N": "mains"}}, "s": "a\"bé"}"#,
        )
        .unwrap();
        let c = &j.path(&["net_settings", "classes"]).unwrap().as_arr()[0];
        assert_eq!(c.get("clearance").unwrap().as_f64(), Some(0.2));
        assert_eq!(j.get("s").unwrap().as_str(), Some("a\"bé"));
        assert_eq!(
            j.path(&["net_settings", "netclass_assignments", "/N"])
                .unwrap()
                .as_str(),
            Some("mains")
        );
    }
}
