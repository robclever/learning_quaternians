use super::model::Demo;
use crate::constants::VISUALIZATION_OUTPUT_FILE_NAME;
use std::{
    fs, io,
    path::{Path, PathBuf},
    process::Command,
};

/// Directory holding generated visualizations (created on demand).
///
/// It lives at the project root so the page is easy to find and share, and it is
/// listed in `.gitignore` so generated output never lands in a commit.
pub fn output_directory() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("visualizations")
}

/// Where the generated demonstration page is written by default.
pub fn default_output_path() -> PathBuf {
    output_directory().join(VISUALIZATION_OUTPUT_FILE_NAME)
}

/// Render the demonstration and write it to `path`.
///
/// Missing parent directories are created, so the caller can pass any path.
/// Returns the path that was written to.
pub fn write_html_file(demo: &Demo, path: &Path) -> io::Result<PathBuf> {
    if let Some(parent) = path.parent() {
        if !parent.as_os_str().is_empty() {
            fs::create_dir_all(parent)?;
        }
    }
    fs::write(path, render_html(demo))?;
    Ok(path.to_path_buf())
}

/// Hand a generated page to the operating system's default browser.
///
/// The browser is spawned, not waited on, so the demo never blocks the CLI.
pub fn open_in_browser(path: &Path) -> io::Result<()> {
    browser_command().arg(path).spawn().map(|_child| ())
}

/// Platform browser launcher used by `open_in_browser`.
#[cfg(target_os = "macos")]
fn browser_command() -> Command {
    Command::new("open")
}

/// Platform browser launcher used by `open_in_browser`.
#[cfg(target_os = "windows")]
fn browser_command() -> Command {
    let mut command = Command::new("cmd");
    // The empty argument after `start` stops Windows from treating the file path
    // as a window title.
    command.args(["/C", "start", ""]);
    command
}

/// Platform browser launcher used by `open_in_browser`.
#[cfg(not(any(target_os = "macos", target_os = "windows")))]
fn browser_command() -> Command {
    Command::new("xdg-open")
}

/// Render the complete, self-contained HTML page for a demonstration.
///
/// The page embeds the frame data as JSON and does no mathematics of its own: it
/// only moves, colours and prints the numbers Rust already computed.
pub fn render_html(demo: &Demo) -> String {
    let mut html = String::with_capacity(64 * 1024);
    html.push_str(HTML_DOCUMENT_HEAD);
    html.push_str(HTML_PAGE_BODY);
    html.push_str(DATA_SCRIPT_OPEN);
    html.push_str(&json_payload(demo));
    html.push_str(HTML_BETWEEN);
    html.push_str(RENDERER_SCRIPT);
    html.push_str(HTML_TAIL);
    html
}

/// Serialize the demonstration as JSON that is safe to inline in a page.
///
/// `<` is escaped so a string in the data can never close the surrounding
/// `<script>` element early.
fn json_payload(demo: &Demo) -> String {
    let json = serde_json::to_string(demo).expect("the demo scene is always serializable");
    json.replace('<', "\\u003c")
}

// Assets are embedded at compile time; the exported page remains self-contained.
const HTML_DOCUMENT_HEAD: &str = concat!(
    include_str!("assets/head.html"),
    "<style>\n",
    include_str!("assets/style.css"),
    "</style>\n</head>\n<body>\n"
);
const HTML_PAGE_BODY: &str = include_str!("assets/body.html");
const DATA_SCRIPT_OPEN: &str = "<script id=\"demo-data\" type=\"application/json\">\n";
const HTML_BETWEEN: &str = "</script>\n";
const RENDERER_SCRIPT: &str = concat!(
    "<script>\n",
    include_str!("assets/renderer.js"),
    "</script>\n"
);
const HTML_TAIL: &str = "</body>\n</html>\n";

#[cfg(test)]
mod tests {
    use super::super::demo::build_gimbal_lock_demo;
    use super::*;
    /// The page must embed the frames as valid JSON and must not be able to break
    /// out of the surrounding script element.
    #[test]
    fn html_embeds_parseable_frame_data() {
        let demo = build_gimbal_lock_demo();
        let html = render_html(&demo);
        assert!(html.starts_with("<!DOCTYPE html>"));
        assert!(html.contains(HTML_PAGE_BODY));

        let start = html.find(DATA_SCRIPT_OPEN).expect("data script tag") + DATA_SCRIPT_OPEN.len();
        let end = start + html[start..].find("</script>").expect("data script end");
        let payload = &html[start..end];
        assert!(
            !payload.contains('<'),
            "the data must not be able to close the script element early"
        );

        let parsed: serde_json::Value = serde_json::from_str(payload).expect("valid demo JSON");
        assert_eq!(
            parsed["frames"].as_array().expect("frames array").len(),
            demo.frames.len()
        );
        assert_eq!(
            parsed["equivalence"]
                .as_array()
                .expect("equivalence array")
                .len(),
            demo.equivalence.len()
        );
        assert_eq!(parsed["frames"][0]["metrics"]["eulerDegrees"][1], 0.0);
    }
    /// Writing the page must create missing directories and produce a real file.
    #[test]
    fn writing_the_page_creates_directories() {
        let demo = build_gimbal_lock_demo();
        let directory = std::env::temp_dir().join("learning_quaternians_visualization_test");
        let path = directory
            .join("nested")
            .join(VISUALIZATION_OUTPUT_FILE_NAME);

        let written = write_html_file(&demo, &path).expect("write the demo page");
        let length = fs::metadata(&written).expect("page metadata").len();
        assert!(length > 20_000, "unexpectedly small page: {length} bytes");

        let _ = fs::remove_dir_all(&directory);
    }
}
