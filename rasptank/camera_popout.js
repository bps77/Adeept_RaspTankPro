function render({ model, el }) {
  const image = document.createElement("img");
  image.alt = "Tank camera";
  image.style.maxWidth = "100%";
  image.style.display = "block";
  const button = document.createElement("button");
  button.textContent = "Pop out video";
  const notice = document.createElement("span");
  notice.setAttribute("role", "status");
  el.append(image, button, notice);

  let popup = null;
  let popupImage = null;
  let popupStatus = null;
  let url = null;
  let disposed = false;

  function updateStatus() {
    if (popup && !popup.closed) {
      popupStatus.textContent = model.get("message");
    }
  }

  function updateFrame() {
    const value = model.get("value");
    if (!value || !value.byteLength) return;
    const next = URL.createObjectURL(new Blob([value], { type: "image/jpeg" }));
    image.src = next;
    if (popup && !popup.closed) popupImage.src = next;
    if (url) URL.revokeObjectURL(url);
    url = next;
  }

  function openPopup() {
    if (popup && !popup.closed) {
      popup.focus();
      return;
    }
    try {
      popup = window.open("", "_blank", "popup,width=800,height=650");
      if (!popup) {
        notice.textContent = " Pop-up blocked. Allow pop-ups for this notebook and retry.";
        return;
      }
      const doc = popup.document;
      doc.title = "Tank camera - video only";
      doc.body.style.cssText = "margin:0;padding:12px;background:#111;color:#fff;font-family:sans-serif";
      popupImage = doc.createElement("img");
      popupImage.alt = "Tank camera";
      popupImage.style.cssText = "display:block;width:100%;height:auto";
      popupStatus = doc.createElement("p");
      popupStatus.setAttribute("role", "status");
      const safety = doc.createElement("p");
      safety.textContent = "Video only. Closing this window does not stop the tank. Use tank.stop() in the notebook.";
      doc.body.replaceChildren(popupImage, popupStatus, safety);
      if (url) popupImage.src = url;
      updateStatus();
      popup.addEventListener("pagehide", () => {
        if (!disposed) notice.textContent = " Pop-out closed; inline video is still available.";
      });
      notice.textContent = " Pop-out open; notebook controls remain available.";
    } catch (error) {
      if (popup && !popup.closed) popup.close();
      popup = null;
      notice.textContent = ` Pop-out failed: ${error.message}. Inline video remains available.`;
    }
  }

  button.addEventListener("click", openPopup);
  model.on("change:value", updateFrame);
  model.on("change:message", updateStatus);
  updateFrame();

  function dispose() {
    if (disposed) return;
    disposed = true;
    window.removeEventListener("pagehide", dispose);
    button.removeEventListener("click", openPopup);
    model.off("change:value", updateFrame);
    model.off("change:message", updateStatus);
    if (popup && !popup.closed) popup.close();
    image.removeAttribute("src");
    if (url) URL.revokeObjectURL(url);
    el.replaceChildren();
  }
  window.addEventListener("pagehide", dispose);
  return dispose;
}

export default { render };
