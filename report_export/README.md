# Application-layer validation export

This bundle contains the Chapter 5 application-layer validation section and its selected figures. Transfer it to the Prism project as follows:

1. Copy `report_export/sections/application_layer_validation/` into the Prism project's `sections/application_layer_validation/` folder.
2. Copy `report_export/images/application_layer_validation/` into the Prism project's `images/application_layer_validation/` folder.
3. Add the following line at the correct location in the Chapter 5 file:

   ```latex
   \input{sections/application_layer_validation/application_layer_validation}
   ```

4. Compile the report from the Prism project root.

The Prism project must already support `\usepackage{graphicx}`, `\usepackage{float}`, and the `\IfFileExists` command used by the existing report. No preamble change is requested unless compilation shows that a required package is missing.

`data_checks.md` records the source-data checks, while `figure_sources.md` records the provenance of each exported figure. The LaTeX file uses paths relative to the Prism project root and prefers the PDF figures. The PNG copies are included as portable alternatives.
